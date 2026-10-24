/*
 *  NfxReverb: a portable C++ port of NFX-RV, the Machinedrum neighbour plate reverb.
 *
 *  Mono Dattorro plate (J. Dattorro, "Effect Design, Part 1", JAES 45(9), 1997,
 *  Fig. 1 and Table 2), rebuilt for mono operation as in the NFX-RV v3 kernel:
 *
 *    1. predelay
 *    2. input bandwidth one-pole (fixed)
 *    3. four input allpasses (0.75, 0.75, 0.625, 0.625)
 *    4. the figure-eight tank: two cross-coupled branches, each with a
 *       modulated allpass (-0.70), a long delay, a damping one-pole, the decay
 *       gain, a second allpass (decay diffusion 2) and a final delay
 *    5. the output sums: Dattorro's seven left taps (NFX-RV's mono output),
 *       and, for stereo, his seven right taps
 *
 *  Delay lengths are Dattorro's x 0.90, given in ticks of a 22.05 kHz tank.
 *  The tank runs at a selectable rate (11.025, 22.05 or 44.1 kHz), like a
 *  tape-speed change: every length stays the same in ticks, so the room, decay
 *  time, predelay and modulation scale with the rate.
 *
 *  At a host sample rate of 44.1 kHz this is NFX-RV tick for tick, in float:
 *  the same averaging of the input into ticks, the same tick order, the same
 *  held-and-smoothed output. It leaves out what only existed for the
 *  fixed-point DSP:
 *    - the predelay's 12-bit packing (about -72 dBFS of quantisation);
 *    - 24-bit saturation inside the tank;
 *    - the per-block (16-tick) linear interpolation of the LFO (here the
 *      triangle is evaluated every tick).
 *  Tank-rate filters: when the tank runs below the host rate (N > 1 host
 *  samples per tick), the MD averages the input over the N samples of a tick,
 *  which lets content above the tank's Nyquist fold back into the tank, and
 *  holds each output tick for N samples, which leaves images above it. The port
 *  adds one 2-pole Butterworth lowpass (a single biquad) at the host rate on
 *  each side: on the tank input before the averaging (anti-alias), and on the
 *  wet output after NFX-RV's hold and half-band poles (anti-image, one per
 *  output channel). Both cutoffs track the tank rate (TANK_FILTER_CUTOFF of the
 *  tick rate). They are bypassed when every host sample is a tick (N = 1) and
 *  can be switched off (setAntiAlias(false), setAntiImage(false)) to compare
 *  with the MD.
 *  At other host rates the tank runs at host_rate / N (N the integer nearest
 *  host_rate / tank_rate) and every time constant is rescaled, so it sounds the
 *  same; at 44.1 kHz no rescaling happens.
 *
 *  Levels: the class works on any scale (it is linear apart from nothing:
 *  there is no internal clipping). NFX-RV's input headroom (/8) and output
 *  gain (x4 on a quarter-scaled tap sum) are kept, so for Machinedrum-equal
 *  levels feed it +-1.0 full scale; in VCV Rack, dividing +-5 V by 5 and
 *  multiplying the output by 5 does that.
 *
 *  Usage (one instance per channel):
 *
 *    nfx::NfxReverb rv(sampleRate);       // or rv.setSampleRate(sr) later
 *    rv.setDecay(d); rv.setMix(m); ...    // all knobs 0..1 (= knob/127 on the MD)
 *    float y = rv.process(x);             // once per sample: mono (the MD's output)
 *    rv.processStereo(x, yL, yR);         // or stereo: yL is the same as process()
 *
 *  Setters are cheap and may be called every sample. setSize() and
 *  setTankRate() recompute the delay table only when the value changes.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace nfx {

// A 2-pole lowpass (RBJ biquad) in transposed direct form II, used for the
// tank-rate filters. It is stable whenever its poles are (|a2| < 1,
// |a1| < 1 + a2), which holds for every cutoff below Nyquist and Q > 0.
struct Lowpass2 {
  float b0 = 1.f, b1 = 0.f, b2 = 0.f, a1 = 0.f, a2 = 0.f, z1 = 0.f, z2 = 0.f;
  void set(float cutoff, float rate, float q) {
    const float pi = 3.14159265358979f;
    float w = 2.f * pi * std::min(cutoff, 0.49f * rate) / rate;
    float cw = std::cos(w), alpha = std::sin(w) / (2.f * q);
    float a0 = 1.f + alpha;
    b0 = (1.f - cw) / (2.f * a0);
    b1 = (1.f - cw) / a0;
    b2 = b0;
    a1 = -2.f * cw / a0;
    a2 = (1.f - alpha) / a0;
  }
  void clear() { z1 = z2 = 0.f; }
  float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};


class NfxReverb {
public:
  enum TankRate { TANK_11K = 0, TANK_22K = 1, TANK_44K = 2 };

  explicit NfxReverb(float sampleRate = 44100.f) { setSampleRate(sampleRate); }

  // ---- Configuration -------------------------------------------------------

  // Allocates the delay lines for this host rate and clears the reverb.
  void setSampleRate(float sampleRate) {
    hostRate = std::max(1000.f, sampleRate);
    // Size every line for the largest rescaling any tank rate can need here.
    float maxScale = 1.f;
    for (int r = 0; r < 3; r++)
      maxScale = std::max(maxScale, timeScaleFor(r));
    for (int i = 0; i < NUM_LINES; i++) {
      int need = (int)std::ceil(LEN[i] * maxScale) + 2 * (int)std::ceil(EXC * maxScale) + 4;
      if (i == PRE)
        need = (int)std::ceil(PRE_MAX_TICKS * maxScale) + 4;
      lines[i].allocate(need);
    }
    rateDirty = true;
    reset();
  }

  // Clears all history and filter states (the MD restarts its history when the
  // machine is initialised).
  void reset() {
    for (auto &l : lines)
      l.clear();
    tick = 0;
    bwState = dampA = dampB = 0.f;
    lfoPhase = 0.0;
    pole1 = pole2 = pole1R = pole2R = 0.f;
    accIn = 0.f;
    inCount = 0;
    heldWet = heldWetR = 0.f;
    aa.clear();
    aiL.clear();
    aiR.clear();
  }

  // ---- Parameters (0..1, the Machinedrum knob / 127) ------------------------

  // DCAY: tank decay, 0..0.961 per gain stage (two stages per circulation).
  void setDecay(float p) {
    float knob = knobOf(p);
    decay = knob * 63488.f / 8388608.f;
    decay2 = std::min(0.5f, std::max(0.25f, decay + 0.15f));   // decay diffusion 2
  }
  // MIX: dry/wet; 1 is wet only.
  void setMix(float p) { mix = levelOf(p); }
  // GAIN: input level into the tank (1 = unity before the internal /8 headroom).
  void setGain(float p) { gain = levelOf(p); }
  // DAMP: the tank's damping one-pole, higher = brighter.
  void setDamp(float p) {
    dampKnobCoef = (knobOf(p) + 0.5f) / 128.f;
    dampDirty = true;
  }
  // BW: on the MD this knob selects the tank rate: 0..42 11.025 kHz,
  // 43..85 22.05 kHz, 86..127 44.1 kHz.
  void setBandwidth(float p) {
    int raw = knobStep(p) * 128;
    setTankRate(raw < 5440 ? TANK_11K : (raw < 10944 ? TANK_22K : TANK_44K));
  }
  void setTankRate(int r) {
    r = std::min(2, std::max(0, r));
    if (r != rate) {
      rate = r;
      rateDirty = true;
    }
  }
  // PRE: predelay, 0.05..100 ms at the 22.05 kHz tank (it scales with the rate).
  void setPredelay(float p) {
    int raw = knobStep(p) * 128;
    preTicksNominal = 1 + ((raw * 2220) >> 14);
    preDirty = true;
  }
  // MOD: tank allpass modulation depth, up to +-8 tank ticks.
  void setMod(float p) { modDepthNominal = 8.f * knobOf(p) / 128.f; }
  // SIZE: scales every tank delay and the output taps, from their minimums at
  // 0 to 100 % at 1. The predelay and input diffusers are not scaled.
  void setSize(float p) {
    float s = std::min(1.f, std::max(0.f, p));
    if (s != size) {
      size = s;
      sizeDirty = true;
    }
  }

  // Machinedrum character: NFX-RV packs its predelay as 12-bit samples, which
  // truncates the tank input toward zero to 12 bits (at +-1 full scale, after
  // the /8 headroom). Off by default; on, the port matches the MD more closely
  // (a little grit on quiet tails).
  void setPredelay12Bit(bool on) { pre12 = on; }

  // The tank-rate filters (see the top), on by default: the anti-alias
  // lowpass on the tank input and the anti-image lowpass on the wet output.
  void setAntiAlias(bool on) {
    if (on != antiAlias) {
      antiAlias = on;
      rateDirty = true;
    }
  }
  void setAntiImage(bool on) {
    if (on != antiImage) {
      antiImage = on;
      rateDirty = true;
    }
  }

  // ---- Processing ----------------------------------------------------------

  // Mono: Dattorro's left taps, exactly NFX-RV's output.
  float process(float in) {
    float l, r;
    step(in, l, r, false);
    return l;
  }

  // Stereo: Dattorro's left and right taps. The left output equals process();
  // the dry share of MIX goes to both.
  void processStereo(float in, float &outL, float &outR) { step(in, outL, outR, true); }

private:
  void step(float in, float &outL, float &outR, bool stereo) {
    if (rateDirty)
      updateRate();
    if (sizeDirty)
      updateSize();
    if (preDirty)
      updatePredelay();
    if (dampDirty)
      updateDamp();

    // First sample of a tick: the output taps for this tick. They reach at
    // least 16 ticks back, so they never depend on this tick's input.
    if (inCount == 0) {
      heldWet = taps(TAPS, tapDelay);
      heldWetR = stereo ? taps(TAPS_R, tapDelayR) : heldWet;
    }

    accIn += aaActive ? aa.process(in) : in;
    inCount++;

    float wet = smooth(heldWet, pole1, pole2);
    float wetR = stereo ? smooth(heldWetR, pole1R, pole2R) : wet;
    if (aiActive) {
      wet = aiL.process(wet);
      wetR = stereo ? aiR.process(wetR) : wet;
    }

    // Last sample of a tick: run the tick on the averaged input.
    if (inCount >= samplesPerTick) {
      float u = gain * (accIn / (float)samplesPerTick) * 0.125f;
      runTick(u);
      accIn = 0.f;
      inCount = 0;
    }

    float dry = in * (1.f - mix);
    outL = dry + mix * wet;
    outR = dry + mix * wetR;
  }

  // At 44.1 kHz the taps go out directly; slower tanks hold each tick and
  // smooth it with NFX-RV's two half-band poles (DC gain 4).
  float smooth(float w, float &p1, float &p2) const {
    if (ticksPerSample1) {
      p1 = p2 = w;
      return 4.f * w;
    }
    float wet = w + p1 + 2.f * p2;
    float n1 = 0.5f * (w + p1);
    float n2 = 0.25f * (w + p1 + 2.f * p2);
    p1 = n1;
    p2 = n2;
    return wet;
  }

  // ---- Fixed design --------------------------------------------------------

  enum Line { PRE, IN1, IN2, IN3, IN4, APA, LONGA, AP2A, POSTA, APB, LONGB, AP2B, POSTB, NUM_LINES };
  // Dattorro's Table 2 lengths x 22050/29761 x 0.90, in 22.05 kHz ticks.
  static constexpr int LEN[NUM_LINES] = {
    0, 95, 71, 253, 185,            // predelay (variable), in1..in4
    448, 2969, 1200, 2481,          // apA, longA, ap2A, postA
    605, 2812, 1771, 2109           // apB, longB, ap2B, postB
  };
  static constexpr int PRE_MAX_TICKS = 2206;   // 100 ms at 22.05 kHz
  static constexpr float EXC = 8.f;            // +-8 ticks of modulation
  static constexpr int DMIN = 16;              // every scaled read
  static constexpr int DMIN_MOD = 26;          // the modulated allpasses
  static constexpr int DMIN_TAP44 = 32;        // NFX-RV's taps at the 44.1 kHz tank
  static constexpr float G_IN[4] = {0.75f, 0.75f, 0.625f, 0.625f};
  static constexpr float G_DIFF1 = -0.70f;
  static constexpr float BW_K = 0.75390625f;   // the input one-pole (old BW default 96)
  static constexpr double LFO_INC = 256.0 / 16777216.0;   // triangle period 65,536 ticks
  static constexpr float BUTTERWORTH_Q = 0.70710678f;
  static constexpr float TANK_FILTER_CUTOFF = 0.35f;   // both filters' cutoff, a fraction of the tick rate (0.7 of its Nyquist)


  struct Tap { int line; int len; float sign; };
  static constexpr int NUM_TAPS = 7;
  // Dattorro's left output, mono: line, delay (22.05 kHz ticks), sign.
  static constexpr Tap TAPS[NUM_TAPS] = {
    {LONGB, 177, 1.f}, {LONGB, 1983, 1.f}, {AP2B, 1276, -1.f}, {POSTB, 1331, 1.f},
    {LONGA, 1327, -1.f}, {AP2A, 125, -1.f}, {POSTA, 711, -1.f}
  };
  // Dattorro's right output (Table 2: +24[353], +24[3627], -31[1228], +33[2673],
  // -48[2111], -55[335], -59[121]), in the same ticks. Not on the MD.
  static constexpr Tap TAPS_R[NUM_TAPS] = {
    {LONGA, 235, 1.f}, {LONGA, 2419, 1.f}, {AP2A, 819, -1.f}, {POSTA, 1782, 1.f},
    {LONGB, 1408, -1.f}, {AP2B, 223, -1.f}, {POSTB, 81, -1.f}
  };

  struct DelayLine {
    std::vector<float> buf;
    uint32_t mask = 0;
    void allocate(int minLen) {
      uint32_t n = 1;
      while (n < (uint32_t)minLen)
        n <<= 1;
      buf.assign(n, 0.f);
      mask = n - 1;
    }
    void clear() { std::fill(buf.begin(), buf.end(), 0.f); }
    float read(uint32_t t, int d) const { return buf[(t - (uint32_t)d) & mask]; }
    void write(uint32_t t, float v) { buf[t & mask] = v; }
  };

  // ---- State -----------------------------------------------------------------

  float hostRate = 44100.f;
  DelayLine lines[NUM_LINES];
  uint32_t tick = 0;
  float bwState = 0.f, dampA = 0.f, dampB = 0.f;
  double lfoPhase = 0.0;          // 0..1, a 24-bit phase on the MD
  float pole1 = 0.f, pole2 = 0.f, pole1R = 0.f, pole2R = 0.f;
  float accIn = 0.f;
  int inCount = 0;
  float heldWet = 0.f, heldWetR = 0.f;
  Lowpass2 aa, aiL, aiR;            // anti-alias (input), anti-image (left, right output)
  bool aaActive = false, aiActive = false;

  // Parameters.
  float decay = 96 * 63488.f / 8388608.f, decay2 = 0.5f;
  float mix = 1.f, gain = 96.f / 128.f;
  float dampKnobCoef = 96.5f / 128.f;
  int rate = TANK_22K;
  int preTicksNominal = 1;
  float modDepthNominal = 4.f;
  float size = 1.f;
  bool pre12 = false;
  bool antiAlias = true;
  bool antiImage = true;

  // Derived.
  bool rateDirty = true, sizeDirty = true, preDirty = true, dampDirty = true;
  int samplesPerTick = 2;
  bool ticksPerSample1 = false;
  float timeScale = 1.f;
  bool exact = true;
  int lineDelay[NUM_LINES] = {};
  int tapDelay[NUM_TAPS] = {}, tapDelayR[NUM_TAPS] = {};
  int preTicks = 1;
  float dampCoef = 96.5f / 128.f, bwCoef = BW_K;
  double lfoInc = LFO_INC;

  // ---- Helpers ---------------------------------------------------------------

  static float knobOf(float p) { return 127.f * std::min(1.f, std::max(0.f, p)); }
  // The nearest whole knob value, for the MD's stepped laws (BW, PRE).
  static int knobStep(float p) { return (int)std::lround(knobOf(p)); }
  // MIX and GAIN: knob/128, with 127 exactly 1.
  static float levelOf(float p) {
    float knob = knobOf(p);
    return knob >= 126.5f ? 1.f : knob / 128.f;
  }

  float nominalRate(int r) const { return r == TANK_11K ? 11025.f : (r == TANK_22K ? 22050.f : 44100.f); }
  int samplesPerTickFor(int r) const { return std::max(1, (int)std::lround(hostRate / nominalRate(r))); }
  float timeScaleFor(int r) const { return hostRate / samplesPerTickFor(r) / nominalRate(r); }

  int scaleTicks(int d) const { return exact ? d : std::max(1, (int)std::lround(d * timeScale)); }
  // A one-pole coefficient per tick, kept at the same cutoff when ticks get shorter or longer.
  float warpCoef(float k) const { return exact ? k : 1.f - std::pow(1.f - k, 1.f / timeScale); }

  void updateRate() {
    samplesPerTick = samplesPerTickFor(rate);
    ticksPerSample1 = samplesPerTick == 1;
    timeScale = timeScaleFor(rate);
    exact = std::fabs(timeScale - 1.f) < 1e-6f;
    lfoInc = LFO_INC / timeScale;
    bwCoef = warpCoef(BW_K);
    // Both tank-rate filters follow the tick rate the tank actually runs at.
    float cutoff = TANK_FILTER_CUTOFF * hostRate / (float)samplesPerTick;
    aaActive = antiAlias && samplesPerTick > 1;
    aiActive = antiImage && samplesPerTick > 1;
    aa.set(cutoff, hostRate, BUTTERWORTH_Q);
    aiL.set(cutoff, hostRate, BUTTERWORTH_Q);
    aiR.set(cutoff, hostRate, BUTTERWORTH_Q);
    rateDirty = false;
    sizeDirty = preDirty = dampDirty = true;
    accIn = 0.f;
    inCount = 0;
  }

  void updateSize() {
    // NFX-RV: d' = max(d_min, round(d * S)), S = SIZE / 127.
    for (int i = IN1; i <= IN4; i++)
      lineDelay[i] = scaleTicks(LEN[i]);
    for (int i = APA; i < NUM_LINES; i++) {
      int dmin = (i == APA || i == APB) ? DMIN_MOD : DMIN;
      lineDelay[i] = scaleTicks(std::max(dmin, (int)std::lround(LEN[i] * size)));
    }
    for (int i = 0; i < NUM_TAPS; i++) {
      tapDelay[i] = tapTicks(TAPS[i].len);
      tapDelayR[i] = tapTicks(TAPS_R[i].len);
    }
    sizeDirty = false;
  }

  int tapTicks(int len) const {
    int d = std::max(DMIN, (int)std::lround(len * size));
    if (rate == TANK_44K)
      d = std::max(DMIN_TAP44, d);
    return scaleTicks(d);
  }

  void updatePredelay() {
    preTicks = scaleTicks(preTicksNominal);
    preDirty = false;
  }

  void updateDamp() {
    dampCoef = warpCoef(dampKnobCoef);
    dampDirty = false;
  }

  float taps(const Tap *tp, const int *delay) const {
    float acc = 0.f;
    for (int i = 0; i < NUM_TAPS; i++)
      acc += tp[i].sign * lines[tp[i].line].read(tick, delay[i]);
    return 0.25f * acc;
  }

  // Schroeder allpass as NFX-RV computes it: w = x - g d, y = g w + d.
  float allpass(int line, float x, float g, int d) {
    float dv = lines[line].read(tick, d);
    float w = x - g * dv;
    lines[line].write(tick, w);
    return g * w + dv;
  }

  // The modulated tank allpass: the delay in fractional ticks, linear interpolation.
  float modAllpass(int line, float x, float delay) {
    int di = (int)std::floor(delay);
    float f = delay - (float)di;
    float s0 = lines[line].read(tick, di);
    float s1 = lines[line].read(tick, di + 1);
    float dv = s0 + f * (s1 - s0);
    float w = x - G_DIFF1 * dv;
    lines[line].write(tick, w);
    return G_DIFF1 * w + dv;
  }

  void runTick(float u) {
    // Front: predelay, bandwidth one-pole, four input diffusers.
    if (pre12)
      u = std::trunc(u * 2048.f) / 2048.f;
    lines[PRE].write(tick, u);
    float v = lines[PRE].read(tick, preTicks);
    bwState += bwCoef * (v - bwState);
    v = bwState;
    for (int i = 0; i < 4; i++)
      v = allpass(IN1 + i, v, G_IN[i], lineDelay[IN1 + i]);

    // The LFO: a triangle in +-1 on the tick's phase (the MD's first tick uses
    // phase 256/2^24, so the phase advances first).
    lfoPhase += lfoInc;
    lfoPhase -= std::floor(lfoPhase);
    double ph = lfoPhase >= 0.5 ? lfoPhase - 1.0 : lfoPhase;      // signed, -0.5..0.5
    float tri = (float)(std::fabs(ph) * 4.0 - 1.0);                 // -1..1
    float lfo = tri * modDepthNominal * (exact ? 1.f : timeScale);

    // Tank: both branches read the other's final delay first (>= 16 ticks back).
    float postA = lines[POSTA].read(tick, lineDelay[POSTA]);
    float postB = lines[POSTB].read(tick, lineDelay[POSTB]);
    branch(v + decay * postB, APA, LONGA, AP2A, POSTA, dampA, lfo);
    branch(v + decay * postA, APB, LONGB, AP2B, POSTB, dampB, -lfo);

    tick++;
  }

  void branch(float x, int ap, int lng, int ap2, int post, float &damp, float lfo) {
    float y = modAllpass(ap, x, (float)lineDelay[ap] + lfo);
    lines[lng].write(tick, y);
    float v = lines[lng].read(tick, lineDelay[lng]);
    damp += dampCoef * (v - damp);
    v = decay * damp;
    v = allpass(ap2, v, decay2, lineDelay[ap2]);
    lines[post].write(tick, v);
  }
};

}  // namespace nfx
