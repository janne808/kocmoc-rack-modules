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
#include "phasor.h"

struct PHASR : Module {
  enum ParamIds {
     TUNE_PARAM,
     FINE_TUNE_PARAM,
     INDEX_PARAM,
     NUM_PARAMS
  };
  enum InputIds {
     PHASE_MOD_INPUT,
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

  PHASR() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(TUNE_PARAM, 0.f, 1.f, 0.5f, "Frequency tune");
    configParam(FINE_TUNE_PARAM, -1.f, 1.f, 0.f, "Frequency finetune");
    configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "Modulation index");
    configInput(PHASE_MOD_INPUT, "Phase modulation");
    configInput(CV_INPUT, "Pitch CV");
    configOutput(OUTPUT_OUTPUT, "Phasor");
  }

  // create phasor instances
  Phasor phasor[16];
  
  void process(const ProcessArgs& args) override {
    // get channels from primary input 
    int channels = inputs[CV_INPUT].getChannels();

    // process at minimum single monophonic channel
    if(channels == 0){
      channels = 1;
    }
    
    // parameters
    float freq = params[TUNE_PARAM].getValue();
    float fine = params[FINE_TUNE_PARAM].getValue();
    float index = params[INDEX_PARAM].getValue();

    // compute base frequency
    freq = std::pow(2.f, 12.0*freq + 0.1*fine);

    // shape index parameter
    index *= index*index*index;
    
    for(int ii = 0; ii < channels; ii++){
      float cv = inputs[CV_INPUT].getVoltage(ii);
      float phase_mod = inputs[PHASE_MOD_INPUT].getVoltage(ii);
      
      // clip negative cv
      if(cv < 0.0) {
	cv = 0.0;
      }

      // set frequency
      phasor[ii].SetFrequency((double)(freq * std::pow(2.f, cv)));

      // set phase modulation
      phasor[ii].SetPhaseModulation((double)(32.0*index*phase_mod));
    
      // tick state
      phasor[ii].Tick();
    
      // set output
      outputs[OUTPUT_OUTPUT].setVoltage(-((float)(phasor[ii].GetPhase()) - M_PI), ii);
    }
    
    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);
  }

  void onReset() override {
    float sr = APP->engine->getSampleRate();
    for(int ii = 0; ii < 16; ii++){
      phasor[ii].SetPhase(0.f);
      phasor[ii].SetFrequency((double)(440.0));
      phasor[ii].SetSampleRate((double)(sr));
    }
  }
  
  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    for(int ii = 0; ii < 16; ii++){
      phasor[ii].SetSampleRate((double)(sr));
    }
  }
};


struct PHASRWidget : ModuleWidget {
  PHASRWidget(PHASR* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/PHASR.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParam<RoundBlackKnob>(mm2px(Vec(4.94, 16.24)), module, PHASR::TUNE_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(4.94, 35.403)), module, PHASR::FINE_TUNE_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(7.02, 55.103)), module, PHASR::INDEX_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 68.82)), module, PHASR::PHASE_MOD_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 85.327)), module, PHASR::CV_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.281, 103.3)), module, PHASR::OUTPUT_OUTPUT));
  }
};

Model* modelPHASR = createModel<PHASR, PHASRWidget>("PHASR");
