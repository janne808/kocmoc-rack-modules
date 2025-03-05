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

#include <cmath>
#include "plugin.hpp"
#include "phasor.h"

struct OP : Module {
  enum ParamIds {
    SCALE_PARAM,
    OFFSET_PARAM,
    INDEX_PARAM,
    PHASE_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    PHASE_MOD_INPUT,
    RESET_INPUT,
    CV_INPUT,
    NUM_INPUTS
  };
  enum OutputIds {
    OUTPUT_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    NUM_LIGHTS
  };

  OP() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(SCALE_PARAM,  1.f, 48.f, 12.f, "Frequency scale");
    configParam(OFFSET_PARAM, 0.f, 128.f, 36.f, "Frequency offset");
    configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "Modulation index");
    configParam(PHASE_PARAM, -1.f*M_PI, 1.f*M_PI, 0.f, "Phase offset");
    configInput(PHASE_MOD_INPUT, "Phase modulation");
    configInput(RESET_INPUT, "Phase reset");
    configInput(CV_INPUT, "Pitch CV");
    configOutput(OUTPUT_OUTPUT, "Operator");
  }

  // create phasor instances
  Phasor phasor[16];
  
  // reset state handling variables
  float last_reset[16];
  
  void process(const ProcessArgs& args) override {
    // get channels from primary input 
    int channels = inputs[CV_INPUT].getChannels();

    // process at minimum single monophonic channel
    if(channels == 0){
      channels = 1;
    }    
    
    // parameters
    int scale = int(params[SCALE_PARAM].getValue());
    int offset = int(params[OFFSET_PARAM].getValue());
    float index = params[INDEX_PARAM].getValue();
    float phase_offset = params[PHASE_PARAM].getValue();

    // shape index parameter
    index *= index*index*index;
    
    for(int ii = 0; ii < channels; ii++){
      float cv = inputs[CV_INPUT].getVoltage(ii) + (float)(offset)/12.f;
      float phase_mod = inputs[PHASE_MOD_INPUT].getVoltage(ii);
      float reset = inputs[RESET_INPUT].getVoltage(ii);
      
      // apply scale to cv
      cv *= (float)(scale)/12.f;

      // clip negative cv
      if(cv < 0.0) {
	cv = 0.0;
      }

      // detect rising zero crossing as a reset state
      if(last_reset[ii] <= 0.0f && reset > 0.0f){
	phasor[ii].SetPhase((double)(0.0f));
      }

      // store last reset input signal
      last_reset[ii] = reset;
      
      // set operator frequency
      phasor[ii].SetFrequency((double)((440.0/128.0) * std::pow(2.f, cv)));
      
      // set operator phase modulation
      phasor[ii].SetPhaseModulation((double)(32.0*index*phase_mod + phase_offset));
    
      // tick state
      phasor[ii].Tick();
      
      // set output
      outputs[OUTPUT_OUTPUT].setVoltage((float)(10.0*std::sin(phasor[ii].GetPhase())), ii);
    }

    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);
  }

  void onReset() override {
    float sr = APP->engine->getSampleRate();
    for(int ii = 0; ii < 16; ii++){
      phasor[ii].SetPhase(0.f);
      phasor[ii].SetFrequency((double)(440.0/128.0));
      phasor[ii].SetSampleRate((double)(sr));
      last_reset[ii] = 0.f;
    }
  }
  
  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    for(int ii = 0; ii < 16; ii++){
      phasor[ii].SetSampleRate((double)(sr));
    }
  }
};


struct OPWidget : ModuleWidget {
  OPWidget(OP* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/OP.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<Trimpot>(mm2px(Vec(3.72, 55.103)), module, OP::INDEX_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(7.981, 16.04)), module, OP::SCALE_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(7.981, 33.703)), module, OP::OFFSET_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(15.525, 55.103)), module, OP::PHASE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.881, 68.82)), module, OP::PHASE_MOD_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.675, 68.82)), module, OP::RESET_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.981, 86.427)), module, OP::CV_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(12.981, 103.3)), module, OP::OUTPUT_OUTPUT));
  }
};

Model* modelOP = createModel<OP, OPWidget>("OP");
