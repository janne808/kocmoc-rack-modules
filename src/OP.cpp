/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Operator VCV Rack plugin.
 *
 *  Operator VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Operator VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Operator VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include "plugin.hpp"
#include "phasor.h"

struct OP : Module {
  enum ParamIds {
    SCALE_PARAM,
    OFFSET_PARAM,
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

  OP() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(SCALE_PARAM,  1.f, 48.f, 12.f, "Frequency scale");
    configParam(OFFSET_PARAM, 0.f, 128.f, 36.f, "Frequency offset");
    configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "Modulation index");
  }

  // create phasor instance
  Phasor *phasor = new Phasor((double)(0.0), (double)(440.0), (double)(APP->engine->getSampleRate()));
  
  void process(const ProcessArgs& args) override {
    // parameters
    int scale = int(params[SCALE_PARAM].getValue());
    int offset = int(params[OFFSET_PARAM].getValue());
    float index = params[INDEX_PARAM].getValue();

    // inputs
    float cv = inputs[CV_INPUT].getVoltage() + (float)(offset)/12.f;
    float phase_mod = inputs[PHASE_MOD_INPUT].getVoltage();

    // apply scale to cv
    cv *= (float)(scale)/12.f;

    // clip negative cv
    if(cv < 0.0) {
      cv = 0.0;
    }

    // shape index parameter
    index *= index*index*index;
    
    // set operator frequency
    phasor->SetFrequency((double)((440.0/128.0) * std::pow(2.f, cv)));

    // set operator phase modulation
    phasor->SetPhaseModulation((double)(32.0*index*phase_mod));
    
    // tick state
    phasor->Tick();
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage((float)(10.0*std::sin(phasor->GetPhase())));
  }
  
  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    phasor->SetSampleRate((double)(sr));
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
    
    addParam(createParam<Trimpot>(mm2px(Vec(7.02, 55.103)), module, OP::INDEX_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(4.94, 16.24)), module, OP::SCALE_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(4.94, 35.403)), module, OP::OFFSET_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 68.82)), module, OP::PHASE_MOD_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 85.327)), module, OP::CV_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.281, 103.3)), module, OP::OUTPUT_OUTPUT));
  }
};

Model* modelOP = createModel<OP, OPWidget>("OP");
