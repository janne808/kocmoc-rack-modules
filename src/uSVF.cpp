/*
 *  (C) 2025 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

// filter modes
enum uSVFFilterMode {
   USVF_LOWPASS_MODE,
   USVF_BANDPASS_MODE,
   USVF_HIGHPASS_MODE
};

struct uSVF : Module {
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
  float hp[16], bp[16], lp[16];

  // system samplerate
  float sampleRate;
  
  uSVF() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.5f, "Cutoff frequency");
    configParam(RESO_PARAM, 0.f, 1.f, 0.f, "Resonance");
    configParam(GAIN_PARAM, 0.f, 1.f, 0.5f, "Gain");
    configSwitch(MODE_PARAM, 0.f, 2.f, 0.f, "Mode", {"Lowpass", "Bandpass", "Highpass"});
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
      hp[ii] = bp[ii] = lp[ii] = 0.f;
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
    float gainComp = params[GAIN_PARAM].getValue() - 0.5;

    // filter state integration parameters
    float fb, dt, input, out;
    
    // shape panel input for a pseudoexponential response
    cutoff = 0.001+2.25*(cutoff * cutoff * cutoff * cutoff);
    gain *= gain * gain * gain;
    lincv_atten *= lincv_atten*lincv_atten;
    expcv_atten *= expcv_atten*expcv_atten;
    
    // compute gain compensation to normalize output on high drive levels
    if(gainComp < 0.0) {
      gainComp = 0.0;
    }
    gainComp = 5.0 * (1.0 - 2.0 * std::log(1.0 + 0.925*gainComp));

    // feedback amount
    fb = 1.f - (1.15f * reso);
    
    // clamp feedback
    if(fb > 0.9f)
      fb = 0.9f;

    for(int ii = 0; ii < channels; ii++){      
      float channelCutoff = cutoff;
      
      // sum in linear cv
      if(inputs[LINCV_INPUT].getChannels() == 1){
	channelCutoff += 2.0f * lincv_atten*inputs[LINCV_INPUT].getVoltage()/10.f;
      }
      else{
	channelCutoff += 2.0f * lincv_atten*inputs[LINCV_INPUT].getVoltage(ii)/10.f;
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
      input = 0.85f * inputs[INPUT_INPUT].getVoltage(ii) * gain;
      
      dt = 44100.f / (sampleRate * 2.f) * channelCutoff;

      // clamp integration rate
      if(dt > 1.25f)
	dt = 1.25f;
      else if(dt < 0.f)
	dt = 0.f;
      
      // integrate with pseudo oversampling
      for(int jj=0; jj < 2; jj++) {
	hp[ii] = input - lp[ii] - fb * bp[ii];
	bp[ii] += dt * hp[ii];
	bp[ii] = FloatTanhPade23(bp[ii]);
	lp[ii] += dt * bp[ii];
      }
      
      // set output
      switch((uSVFFilterMode)params[MODE_PARAM].getValue()) {
      case USVF_LOWPASS_MODE:
	out = lp[ii];
	break;
      case USVF_BANDPASS_MODE:
	out = bp[ii];
	break;
      case USVF_HIGHPASS_MODE:
	out = hp[ii];
	break;
      default:
	out = 0.f;
      }
      
      outputs[OUTPUT_OUTPUT].setVoltage((float)(2.f * out * gainComp), ii);
    }

    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);    
  }

  void onSampleRateChange() override {
    // new system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii = 0; ii < 16; ii++){    
      hp[ii] = bp[ii] = lp[ii] = 0.f;
    }
  }

  void onReset() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii=0; ii < 16; ii++) {
      hp[ii] = bp[ii] = lp[ii] = 0.f;
    }
  }

  void onAdd() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // reset filter state
    for(int ii=0; ii < 16; ii++) {
      hp[ii] = bp[ii] = lp[ii] = 0.f;
    }
  }  
};

struct uSVFWidget : ModuleWidget {
  uSVFWidget(uSVF* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/uSVF.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(8.84, 13.64)), module, uSVF::FREQ_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(11.24, 33.86)), module, uSVF::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(4.93, 84.38)), module, uSVF::GAIN_PARAM));
    
    addParam(createParam<Trimpot>(mm2px(Vec(5.86, 51.52)), module, uSVF::LINCV_ATTEN_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(18.621, 51.52)), module, uSVF::EXPCV_ATTEN_PARAM));
    
    addParam(createParam<CKSSThree>(Vec(58.48, 248.3), module, uSVF::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 65.52)), module, uSVF::LINCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 65.52)), module, uSVF::EXPCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 104.7)), module, uSVF::INPUT_INPUT));
    
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(21.681, 104.7)), module, uSVF::OUTPUT_OUTPUT));
  }
};

Model* modeluSVF = createModel<uSVF, uSVFWidget>("uSVF");
