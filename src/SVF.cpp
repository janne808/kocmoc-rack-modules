/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
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
#include "svfilter.h"

struct SVF_1 : Module {
  enum ParamIds {
     FREQ_PARAM,
     RESO_PARAM,
     GAIN_PARAM,
     MODE_PARAM,
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

  int _oversampling = 2;
  SVFIntegrationMethod _integrationMethod = SVF_PREDICTOR_CORRECTOR;
  
  // create svf class instance
  SVFilter *svf = new SVFilter((double)(0.25), (double)(0.0), _oversampling, SVF_LOWPASS_MODE,
			       (double)(APP->engine->getSampleRate()), _integrationMethod);
  
  SVF_1() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.5f, "");
    configParam(RESO_PARAM, 0.f, 1.f, 0.f, "");
    configParam(GAIN_PARAM, 0.f, 1.f, 0.5f, "");
    configParam(MODE_PARAM, 0.f, 2.f, 0.f, "");
  }

  void process(const ProcessArgs& args) override {
    // parameters
    float cutoff = params[FREQ_PARAM].getValue();
    float reso = params[RESO_PARAM].getValue();
    float gain = params[GAIN_PARAM].getValue();
    float gainComp;
    
    // shape panel input for a pseudoexponential response
    cutoff = 0.001+2.25*(cutoff * cutoff * cutoff * cutoff);
    gain = (gain * gain * gain * gain)/10.f;
    
    // sum in linear cv
    cutoff += inputs[LINCV_INPUT].getVoltage()/10.f;

    // apply exponential cv
    cutoff = cutoff * std::pow(2.f, inputs[EXPCV_INPUT].getVoltage());
      
    // set filter parameters
    svf->SetFilterCutoff((double)(cutoff));
    svf->SetFilterResonance((double)(reso));
    svf->SetFilterMode((SVFFilterMode)(params[MODE_PARAM].getValue()));
    
    // tick filter state
    svf->filter((double)(inputs[INPUT_INPUT].getVoltage() * gain * 3.0));

    // compute gain compensation to normalize output on high drive levels
    gain = params[GAIN_PARAM].getValue() - 0.5;
    if(gain < 0.0) {
      gain = 0.0;
    }
    gainComp = 9.0 * (1.0 - 1.9 * std::log(1.0 + gain));
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage((float)(svf->GetFilterOutput() * 0.9 * gainComp));
  }

  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    svf->SetFilterSampleRate(sr);
  }

  void onReset() override {
    float sr = APP->engine->getSampleRate();
    svf->ResetFilterState();
    svf->SetFilterOversamplingFactor(_oversampling);
    svf->SetFilterSampleRate(sr);
  }

  void onAdd() override {
    float sr = APP->engine->getSampleRate();
    svf->SetFilterOversamplingFactor(_oversampling);
    svf->SetFilterSampleRate(sr);
    svf->SetFilterIntegrationMethod(_integrationMethod);
  }
  
  json_t* dataToJson() override {
    json_t* rootJ = json_object();
    json_object_set_new(rootJ, "oversampling", json_integer(_oversampling));
    json_object_set_new(rootJ, "integrationMethod", json_integer((int)(_integrationMethod)));
    return rootJ;
  }

  void dataFromJson(json_t* rootJ) override {
    json_t* oversamplingJ = json_object_get(rootJ, "oversampling");
    if (oversamplingJ)
      _oversampling = json_integer_value(oversamplingJ);
    json_t* integrationMethodJ = json_object_get(rootJ, "integrationMethod");
    if (integrationMethodJ)
      _integrationMethod = (SVFIntegrationMethod)(json_integer_value(integrationMethodJ));
  }
};

struct SVFWidget : ModuleWidget {
  SVFWidget(SVF_1* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/SVF.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundHugeBlackKnob>(mm2px(Vec(5.95, 15.34)), module, SVF_1::FREQ_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(10.24, 42.06)), module, SVF_1::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(4.93, 84.38)), module, SVF_1::GAIN_PARAM));

    addParam(createParam<CKSSThree>(Vec(58.48, 248.3), module, SVF_1::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 69.52)), module, SVF_1::LINCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 69.52)), module, SVF_1::EXPCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 104.7)), module, SVF_1::INPUT_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(21.681, 104.7)), module, SVF_1::OUTPUT_OUTPUT));
  }

  struct OversamplingMenuItem : MenuItem {
    SVF_1* _module;
    const int _oversampling;

    OversamplingMenuItem(SVF_1* module, const char* label, int oversampling)
      : _module(module)
      , _oversampling(oversampling)
    {
      this->text = label;
    }

    void onAction(const event::Action& e) override {
      _module->_oversampling = _oversampling;
      _module->svf->SetFilterOversamplingFactor(_module->_oversampling);
    }

    void step() override {
      MenuItem::step();
      rightText = _module->_oversampling == _oversampling ? "✔" : "";
    }
  };
  
  struct IntegrationMenuItem : MenuItem {
    SVF_1* _module;
    const SVFIntegrationMethod _integrationMethod;

    IntegrationMenuItem(SVF_1* module, const char* label, SVFIntegrationMethod integrationMethod)
      : _module(module)
      , _integrationMethod(integrationMethod)
    {
      this->text = label;
    }

    void onAction(const event::Action& e) override {
      _module->_integrationMethod = _integrationMethod;
      _module->svf->SetFilterIntegrationMethod(_module->_integrationMethod);
    }

    void step() override {
      MenuItem::step();
      rightText = _module->_integrationMethod == _integrationMethod ? "✔" : "";
    }
  };
  
  void appendContextMenu(Menu* menu) override {
    SVF_1* a = dynamic_cast<SVF_1*>(module);
    assert(a);
    
    menu->addChild(new MenuEntry());
    menu->addChild(createMenuLabel("Oversampling"));
    menu->addChild(new OversamplingMenuItem(a, "Oversampling: off", 1));
    menu->addChild(new OversamplingMenuItem(a, "Oversampling: x2", 2));
    menu->addChild(new OversamplingMenuItem(a, "Oversampling: x4", 4));
    menu->addChild(new OversamplingMenuItem(a, "Oversampling: x8", 8));

    menu->addChild(new MenuEntry());
    menu->addChild(createMenuLabel("Integration Method"));
    menu->addChild(new IntegrationMenuItem(a, "Semi-implicit Euler", SVF_SEMI_IMPLICIT_EULER));
    menu->addChild(new IntegrationMenuItem(a, "Predictor-Corrector", SVF_PREDICTOR_CORRECTOR));
  }
};

Model* modelSVF_1 = createModel<SVF_1, SVFWidget>("SVF-1");
