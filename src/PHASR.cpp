/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Phasor VCV Rack plugin.
 *
 *  Phasor VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Phasor VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Phasor VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
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
    configParam(TUNE_PARAM, 0.f, 1.f, 0.5f, "");
    configParam(FINE_TUNE_PARAM, -1.f, 1.f, 0.f, "");
    configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "");
  }

  // create phasor instance
  Phasor *phasor = new Phasor((double)(0.0), (double)(440.0), (double)(APP->engine->getSampleRate()));
  
  void process(const ProcessArgs& args) override {
    // parameters
    float freq = params[TUNE_PARAM].getValue();
    float fine = params[FINE_TUNE_PARAM].getValue();
    float index = params[INDEX_PARAM].getValue();

    // compute base frequency
    freq = std::pow(2.f, 12.0*freq + 0.1*fine);

    // shape index parameter
    index *= index*index*index;
    
    // set operator frequency
    phasor->SetFrequency((double)(freq * std::pow(2.f, inputs[CV_INPUT].getVoltage())));

    // set operator phase modulation
    phasor->SetPhaseModulation((double)(64.0*index*inputs[PHASE_MOD_INPUT].getVoltage()));
    
    // tick state
    phasor->Tick();
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage((float)(phasor->GetPhase()) - M_PI);
  }

  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    phasor->SetSampleRate((double)(sr));
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
