/*
 *  (C) 2024 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

struct MUL : Module {
  enum ParamIds {
     CONST1_PARAM,
     CONST2_PARAM,
     NUM_PARAMS
  };
  enum InputIds {
     IN1_1_INPUT,
     IN1_2_INPUT,
     IN2_1_INPUT,
     IN2_2_INPUT,
     NUM_INPUTS
  };
  enum OutputIds {
     OUTPUT_OUTPUT,
     NUM_OUTPUTS
  };
  enum LightIds {
     NUM_LIGHTS
  };

  MUL() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(CONST1_PARAM, -8.f, 8.f, 0.f, "Multiplier amount");
    configParam(CONST2_PARAM, -8.f, 8.f, 0.f, "Multiplier amount");
    configInput(IN1_1_INPUT, "Multiplier");
    configInput(IN2_1_INPUT, "Multiplicant");
    configInput(IN1_2_INPUT, "Multiplier");
    configInput(IN2_2_INPUT, "Multiplicant");
    configOutput(OUTPUT_OUTPUT, "Multiplication");
  }

  void process(const ProcessArgs& args) override {
    float in1_1=1.0, in1_2=1.0;

    // check if inputs are connected and normalize if not
    if(inputs[IN1_1_INPUT].isConnected()) {
      in1_1 = inputs[IN1_1_INPUT].getVoltage();
    }
    if(inputs[IN1_2_INPUT].isConnected()) {
      in1_2 = inputs[IN1_2_INPUT].getVoltage();
    }
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage(params[CONST1_PARAM].getValue()*in1_1*inputs[IN2_1_INPUT].getVoltage() +
				      params[CONST2_PARAM].getValue()*in1_2*inputs[IN2_2_INPUT].getVoltage());    
  }
};


struct MULWidget : ModuleWidget {
  MULWidget(MUL* module) {
    float yOffset = -2.25;
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/MUL.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParam<Trimpot>(mm2px(Vec(7.02, 19.303 + yOffset)), module, MUL::CONST1_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(7.02, 60.303 + yOffset)), module, MUL::CONST2_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 33.02 + yOffset)), module, MUL::IN1_1_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 74.02 + yOffset)), module, MUL::IN1_2_INPUT));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 47.02 + yOffset)), module, MUL::IN2_1_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 88.02 + yOffset)), module, MUL::IN2_2_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.281, 103.3)), module, MUL::OUTPUT_OUTPUT));
  }
};

Model* modelMUL = createModel<MUL, MULWidget>("MUL");
