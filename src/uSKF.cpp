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
#define ERROR_COMPENSATION_FACTOR 0.35f

// define integration rate clamping
#define INTEGRATION_RATE_MAX 0.55f

// define oversampling
#define OVERSAMPLING_FACTOR 2

// define saturation method (0 = dynamic antisaturation, 1 = static saturation)
#define SATURATION_METHOD 0

// filter modes
enum uSKFFilterMode {
   USK_LOWPASS_MODE,
   USK_BANDPASS_MODE,
   USK_HIGHPASS_MODE
};

struct uSKF : Module {
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
  float p0[16], p1[16];
  
  // system samplerate
  float sampleRate;
  
  uSKF() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.5f, "Cutoff frequency");
    configParam(RESO_PARAM, 0.f, 1.f, 0.f, "Resonance");
    configParam(GAIN_PARAM, 0.f, 1.f, 0.5f, "Gain");
    configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"Lowpass", "Bandpass"});
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
    resetFilterState();
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
    float gainComp = params[GAIN_PARAM].getValue() - 0.5;

    // filter state integration parameters
    float fb, dt, error_comp, input, input_lp, input_bp, out;
    
    // shape panel input for a pseudoexponential response
    cutoff = 0.001 + 2.25 * (cutoff * cutoff * cutoff * cutoff);
    gain *= gain * gain * gain;
    lincv_atten *= lincv_atten * lincv_atten;
    expcv_atten *= expcv_atten * expcv_atten;

    // compute gain compensation to normalize output on high drive levels
    if(gainComp < 0.0) {
      gainComp = 0.0;
    }
    
    gainComp = 9.0 * (1.0 - 1.9 * std::log(1.0 + gainComp));
    
    // compute feedback amount
    fb = 3.25f * reso;
    
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
      input = FloatTanhPade23(1.0f * inputs[INPUT_INPUT].getVoltage(ii) * gain);
      
      dt = 44100.f / (sampleRate * (float)(OVERSAMPLING_FACTOR)) * channelCutoff;
	
      // clamp integration rate
      if (dt > INTEGRATION_RATE_MAX) {
	dt = INTEGRATION_RATE_MAX;
      }
      else if(dt < 0.f) {
	dt = 0.f;
      }

      // integration error compensation factor
      error_comp = 1.f + ERROR_COMPENSATION_FACTOR * dt;

      // set input mode
      switch((uSKFFilterMode)params[MODE_PARAM].getValue()){
      case USK_LOWPASS_MODE:
	input_lp = input;
	input_bp = 0.f;
	break;
      case USK_BANDPASS_MODE:
	input_lp = 0.f;
	input_bp = input;
	break;
      default:
	input_lp = input_bp = 0.f;
      }
      
#if SATURATION_METHOD == 0
      // integrate with pseudo oversampling and dynamic antisaturation
      for(int jj=0; jj < OVERSAMPLING_FACTOR; jj++) {
	float feedback = error_comp * (input_bp + fb * p1[ii]);
	
	p0[ii] += dt * (input_lp - p0[ii] - feedback);
       	p1[ii] += dt * (p0[ii] + feedback - p1[ii] - 1.0f / 4.0f * FloatSinhPade54(p1[ii] * 4.0f));
      }
      
#elif SATURATION_METHOD == 1
      // integrate with pseudo oversampling and static saturation
      for(int jj=0; jj < OVERSAMPLING_FACTOR; jj++) {
	float feedback = error_comp * (input_bp + fb * FloatTanhPade23(p1[ii]));

	p0[ii] += dt * (input_lp - p0[ii] - feedback);
       	p1[ii] += dt * (p0[ii] + feedback - p1[ii]);
      }
#endif
      
      // set output
      out = p1[ii];
      
      outputs[OUTPUT_OUTPUT].setVoltage((float)(2.0f * out * gainComp), ii);
    }

    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);    
  }

  void resetFilterState() {
    // reset filter state
    for(int ii = 0; ii < 16; ii++){    
      p0[ii] = p1[ii] = 0.f;
    }
  }
  
  void onSampleRateChange() override {
    // new system samplerate
    sampleRate = APP->engine->getSampleRate();

    
    // reset filter state
    resetFilterState();
  }

  void onReset() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    resetFilterState();
  }

  void onAdd() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    resetFilterState();
  }  
};

struct uSKFWidget : ModuleWidget {
  uSKFWidget(uSKF* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/uSKF.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(8.84, 13.64)), module, uSKF::FREQ_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(11.24, 33.86)), module, uSKF::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(4.93, 84.38)), module, uSKF::GAIN_PARAM));
    
    addParam(createParam<Trimpot>(mm2px(Vec(5.86, 51.52)), module, uSKF::LINCV_ATTEN_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(18.621, 51.52)), module, uSKF::EXPCV_ATTEN_PARAM));
    
    addParam(createParam<CKSS>(Vec(57.0, 252.3), module, uSKF::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 65.52)), module, uSKF::LINCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 65.52)), module, uSKF::EXPCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 104.7)), module, uSKF::INPUT_INPUT));
    
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(21.681, 104.7)), module, uSKF::OUTPUT_OUTPUT));
  }
};

Model* modeluSKF = createModel<uSKF, uSKFWidget>("uSKF");
