/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of State Variable Filter VCV Rack plugin.
 *
 *  State Variable Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  State Variable Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with State Variable Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "plugin.hpp"
#include "svf.h"

struct SVF_1 : Module {
  enum ParamIds {
     FREQ_PARAM,
     RESO_PARAM,
     GAIN_PARAM,
     MODE_PARAM,
     NUM_PARAMS
  };
  enum InputIds {
     CV_INPUT,
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

  // create svf class instance
  SVF *svf = new SVF((double)(0.25), (double)(0.0), 2, 0, (double)(APP->engine->getSampleRate()));
  
  SVF_1() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.5f, "");
    configParam(RESO_PARAM, 0.f, 1.f, 0.f, "");
    configParam(GAIN_PARAM, 0.f, 1.f, 0.25f, "");
    configParam(MODE_PARAM, 0.f, 2.f, 0.f, "");
  }

  void process(const ProcessArgs& args) override {
    // parameter filters
    static float cutoff_prime=0;
    static float reso_prime=0;

    float cutoff = params[FREQ_PARAM].getValue();
    float reso = params[RESO_PARAM].getValue();

    // smooth parameter changes
    cutoff_prime = (cutoff * (1.0-0.999)) + (cutoff_prime * 0.999); 
    reso_prime = (reso * (1.0-0.993)) + (reso_prime * 0.993); 
    
    // set filter parameters
    // sum up CV into cutoff frequency
    svf->SetFilterCutoff((double)(cutoff_prime + inputs[CV_INPUT].getVoltage()/5.f));
    svf->SetFilterResonance((double)(reso_prime));
    svf->SetFilterMode(params[MODE_PARAM].getValue());
    
    // tick filter state
    svf->SVFfilter((double)(inputs[INPUT_INPUT].getVoltage()*params[GAIN_PARAM].getValue()/10.f));
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage((float)(svf->GetFilterOutput()*2.5));
  }

  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    svf->SetFilterSampleRate(sr);
  }

  //void onReset(){
  //}
};

struct SVF_1Widget : ModuleWidget {
  SVF_1Widget(SVF_1* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/SVF-1.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundHugeBlackKnob>(mm2px(Vec(10.68, 30.74)), module, SVF_1::FREQ_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(15.24, 58.06)), module, SVF_1::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(6.18, 81.38)), module, SVF_1::GAIN_PARAM));

    addParam(createParam<CKSSThree>(Vec(83.48, 238.7), module, SVF_1::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 20.42)), module, SVF_1::CV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 101.7)), module, SVF_1::INPUT_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.48, 101.7)), module, SVF_1::OUTPUT_OUTPUT));
  }
};

Model* modelSVF_1 = createModel<SVF_1, SVF_1Widget>("SVF-1");
