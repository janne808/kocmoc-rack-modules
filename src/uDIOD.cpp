/*
 *  (C) 2026 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Kocmoc VCV Rack plugin.
 *
 *  Kocmoc VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Kocmoc VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Kocmoc VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "plugin.hpp"

#include "fastmath.h"

// define integration error compensation factor
#define ERROR_COMPENSATION_FACTOR 2.f

// define integration rate clamping
#define INTEGRATION_RATE_MAX 0.65f

// define oversampling
#define OVERSAMPLING_FACTOR 2

// feedback DC decoupling integration rate
#define FEEDBACK_DC_DECOUPLING_INTEGRATION_RATE 0.005

// filter modes
enum uDIODFilterMode {
   UDIOD_LOWPASS4_MODE,
   UDIOD_LOWPASS2_MODE,
};

struct uDIOD : Module {
  enum ParamIds {
     FREQ_PARAM,
     RESO_PARAM,
     GAIN_PARAM,
     MODE_PARAM,
     LINCV_ATTEN_PARAM,
     EXPCV_ATTEN_PARAM,
     NUM_PARAMS
  };
  enum InputIds {
     LINCV_INPUT,
     EXPCV_INPUT,
     INPUT_INPUT,
     NUM_INPUTS
  };
  enum OutputIds {
     OUTPUT_OUTPUT,
     NUM_OUTPUTS
  };
  enum LightIds {
     NUM_LIGHTS
  };

  // filter state
  float p0[16], p1[16], p2[16], p3[16];
  float hp0[16], hp1[16], hp2[16], hp3[16], hp4[16], hp5[16];
  
  // system samplerate
  float sampleRate;
  
  uDIOD() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.5f, "Cutoff frequency");
    configParam(RESO_PARAM, 0.f, 1.f, 0.f, "Resonance");
    configParam(GAIN_PARAM, 0.f, 1.f, 0.5f, "Gain");
    configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"Lowpass4", "Lowpass2"});
    configParam(LINCV_ATTEN_PARAM, -1.f, 1.f, 0.f, "CV Amount");
    configParam(EXPCV_ATTEN_PARAM, -1.f, 1.f, 0.f, "CV Amount");
    configInput(LINCV_INPUT, "Linear CV");
    configInput(EXPCV_INPUT, "Exponential CV");
    configInput(INPUT_INPUT, "Audio");
    configOutput(OUTPUT_OUTPUT, "Filter");
    configBypass(INPUT_INPUT, OUTPUT_OUTPUT);

    // get system samplerate
    sampleRate = APP->engine->getSampleRate();    
    
    // reset filter state
    for(int ii=0; ii < 16; ii++) {
      p0[ii] = p1[ii] = p2[ii] = p3[ii] = 0.f;
      hp0[ii] = hp1[ii] = hp2[ii] = hp3[ii] = hp4[ii] = hp5[ii] = 0.f;
    }
  }

  void process(const ProcessArgs& args) override {
    // get channels from primary input 
    int channels = inputs[INPUT_INPUT].getChannels();

    // process at minimum single monophonic channel
    if(channels == 0){
      channels = 1;
    }    
    
    // parameters
    float cutoff = params[FREQ_PARAM].getValue();
    float lincv_atten = params[LINCV_ATTEN_PARAM].getValue();
    float expcv_atten = params[EXPCV_ATTEN_PARAM].getValue();
    float reso = params[RESO_PARAM].getValue();
    float gain = params[GAIN_PARAM].getValue();

    // filter state integration parameters
    float fb, dt, dt_hp, error, input, out;
    
    // shape panel input for a pseudoexponential response
    cutoff = 0.001 + 2.25 * (cutoff * cutoff * cutoff * cutoff);
    gain *= gain * gain * gain;
    lincv_atten *= lincv_atten * lincv_atten;
    expcv_atten *= expcv_atten * expcv_atten;

    // compute feedback amount
    fb = 24.0f * reso;
    
    for(int ii = 0; ii < channels; ii++){      
      float channelCutoff = cutoff;

      // sum in linear cv
      if(inputs[LINCV_INPUT].getChannels() == 1){
	channelCutoff += 2.0f * lincv_atten*inputs[LINCV_INPUT].getVoltage() / 10.f;
      }
      else{
	channelCutoff += 2.0f * lincv_atten*inputs[LINCV_INPUT].getVoltage(ii) / 10.f;
      }
      
      // apply exponential cv
      if(inputs[EXPCV_INPUT].getChannels() == 1){
	channelCutoff = channelCutoff * std::pow(2.f, expcv_atten*inputs[EXPCV_INPUT].getVoltage());
      }
      else{
	channelCutoff = channelCutoff * std::pow(2.f, expcv_atten*inputs[EXPCV_INPUT].getVoltage(ii));
      }
      
      // tick filter state
      // with semi-implicit euler integration
      input = 0.8f * inputs[INPUT_INPUT].getVoltage(ii) * gain;
      
      dt = 44100.f / (sampleRate * (float)(OVERSAMPLING_FACTOR)) * channelCutoff;
      dt_hp = 44100.0f / (sampleRate * (float)(OVERSAMPLING_FACTOR)) * FEEDBACK_DC_DECOUPLING_INTEGRATION_RATE;
	
      // clamp integration rate
      if(dt > INTEGRATION_RATE_MAX)
	dt = INTEGRATION_RATE_MAX;
      else if(dt < 0.f)
	dt = 0.f;

      // integration error compensation factor
      error = 1.f + ERROR_COMPENSATION_FACTOR * dt;
      
      // integrate with pseudo oversampling
      for(int jj=0; jj < OVERSAMPLING_FACTOR; jj++) {
	p0[ii] += dt * (FloatTanhPade23(input - error * fb * hp3[ii]) - (p0[ii] - p1[ii]));
	p1[ii] += 0.5f * dt * ((p0[ii] - p1[ii]) - (p1[ii] - p2[ii]));
	p2[ii] += 0.5f * dt * ((p1[ii] - p2[ii]) - (p2[ii] - p3[ii]));
	p3[ii] += 0.5f * dt * ((p2[ii] - p3[ii]) - p3[ii]);

	hp0[ii] = hp0[ii] + dt_hp * (p3[ii] - hp0[ii]);
	hp1[ii] = p3[ii] - hp0[ii];
	
	hp2[ii] = hp2[ii] + dt_hp * (hp1[ii] - hp2[ii]);
	hp3[ii] = hp1[ii] - hp2[ii];

	hp4[ii] = hp4[ii] + dt_hp * (p1[ii] - hp4[ii]);
	hp5[ii] = p1[ii] - hp4[ii];
      }
    
      // set output
      switch((uDIODFilterMode)params[MODE_PARAM].getValue()) {
      case UDIOD_LOWPASS4_MODE:
	out = hp1[ii];
	break;
      case UDIOD_LOWPASS2_MODE:
	out = 0.25f * hp5[ii];
	break;
      default:
	out = 0.f;
      }
      
      outputs[OUTPUT_OUTPUT].setVoltage((float)(24.0f * out), ii);
    }

    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);    
  }

  void onSampleRateChange() override {
    // new system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii = 0; ii < 16; ii++){    
      p0[ii] = p1[ii] = p2[ii] = p3[ii] = 0.f;
    }
  }

  void onReset() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii=0; ii < 16; ii++) {
      p0[ii] = p1[ii] = p2[ii] = p3[ii] = 0.f;
    }
  }

  void onAdd() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii=0; ii < 16; ii++) {
      p0[ii] = p1[ii] = p2[ii] = p3[ii] = 0.f;
    }
  }  
};

struct uDIODWidget : ModuleWidget {
  uDIODWidget(uDIOD* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/uDIOD.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(8.84, 13.64)), module, uDIOD::FREQ_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(11.24, 33.86)), module, uDIOD::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(4.93, 84.38)), module, uDIOD::GAIN_PARAM));
    
    addParam(createParam<Trimpot>(mm2px(Vec(5.86, 51.52)), module, uDIOD::LINCV_ATTEN_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(18.621, 51.52)), module, uDIOD::EXPCV_ATTEN_PARAM));
    
    addParam(createParam<CKSS>(Vec(57.0, 252.3), module, uDIOD::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 65.52)), module, uDIOD::LINCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 65.52)), module, uDIOD::EXPCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 104.7)), module, uDIOD::INPUT_INPUT));
    
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(21.681, 104.7)), module, uDIOD::OUTPUT_OUTPUT));
  }
};

Model* modeluDIOD = createModel<uDIOD, uDIODWidget>("uDIOD");
