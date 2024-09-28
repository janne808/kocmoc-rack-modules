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
#include "svfilter.h"

struct SVF_1 : Module {
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

  int _oversampling = 2;
  int _decimatorOrder = 16;
  
  SVFIntegrationMethod _integrationMethod = SVF_INV_TRAPEZOIDAL;
  
  // create svf class instances
  SVFilter svf[16];
  
  SVF_1() {
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
    
    for(int ii = 0; ii < channels; ii++){      
      float channelCutoff = cutoff;
      
      // sum in linear cv
      if(inputs[LINCV_INPUT].getChannels() == 1){
	channelCutoff += lincv_atten*inputs[LINCV_INPUT].getVoltage()/10.f;
      }
      else{
	channelCutoff += lincv_atten*inputs[LINCV_INPUT].getVoltage(ii)/10.f;
      }
      
      // apply exponential cv
      if(inputs[EXPCV_INPUT].getChannels() == 1){
	channelCutoff = channelCutoff * std::pow(2.f, expcv_atten*inputs[EXPCV_INPUT].getVoltage());
      }
      else{
	channelCutoff = channelCutoff * std::pow(2.f, expcv_atten*inputs[EXPCV_INPUT].getVoltage(ii));
      }
      
      // set filter parameters
      svf[ii].SetFilterCutoff((double)(channelCutoff));
      svf[ii].SetFilterResonance((double)(reso));
      svf[ii].SetFilterMode((SVFFilterMode)(params[MODE_PARAM].getValue()));
    
      // tick filter state
#ifdef FLOATDSP
      svf[ii].filter((float)(inputs[INPUT_INPUT].getVoltage(ii) * gain));
#else
      svf[ii].filter((double)(inputs[INPUT_INPUT].getVoltage(ii) * gain));
#endif
      
      // set output
      outputs[OUTPUT_OUTPUT].setVoltage((float)(svf[ii].GetFilterOutput() * gainComp), ii);
    }

    // set output to be polyphonic
    outputs[OUTPUT_OUTPUT].setChannels(channels);    
  }

  void onSampleRateChange() override {
    float sr = APP->engine->getSampleRate();
    for(int ii = 0; ii < 16; ii++){    
      svf[ii].SetFilterSampleRate(sr);
    }
  }

  void onReset() override {
    float sr = APP->engine->getSampleRate();
    
    for(int ii = 0; ii < 16; ii++){    
      svf[ii].ResetFilterState();
      svf[ii].SetFilterSampleRate(sr);
      svf[ii].SetFilterCutoff((double)(0.25));
      svf[ii].SetFilterResonance((double)(0.0));
      svf[ii].SetFilterMode(SVF_LOWPASS_MODE);
      svf[ii].SetFilterIntegrationMethod(_integrationMethod);
      svf[ii].SetFilterOversamplingFactor(_oversampling);
      svf[ii].SetFilterDecimatorOrder(_decimatorOrder);
    }
  }

  void onAdd() override {
    float sr = APP->engine->getSampleRate();
    
    for(int ii = 0; ii < 16; ii++){    
      svf[ii].ResetFilterState();
      svf[ii].SetFilterSampleRate(sr);
      svf[ii].SetFilterCutoff((double)(0.25));
      svf[ii].SetFilterResonance((double)(0.0));
      svf[ii].SetFilterMode(SVF_LOWPASS_MODE);
      svf[ii].SetFilterIntegrationMethod(_integrationMethod);
      svf[ii].SetFilterOversamplingFactor(_oversampling);
      svf[ii].SetFilterDecimatorOrder(_decimatorOrder);
    }
  }
  
  json_t* dataToJson() override {
    json_t* rootJ = json_object();
    
    json_object_set_new(rootJ, "integrationMethod", json_integer((int)(_integrationMethod)));
    json_object_set_new(rootJ, "oversampling", json_integer(_oversampling));
    json_object_set_new(rootJ, "decimatorOrder", json_integer(_decimatorOrder));
    
    return rootJ;
  }

  void dataFromJson(json_t* rootJ) override {
    json_t* integrationMethodJ = json_object_get(rootJ, "integrationMethod");
    if (integrationMethodJ && (_integrationMethod != (SVFIntegrationMethod)(json_integer_value(integrationMethodJ)))) {
      _integrationMethod = (SVFIntegrationMethod)(json_integer_value(integrationMethodJ));
      
      // set new integration method
      for(int ii = 0; ii < 16; ii++)
	svf[ii].SetFilterIntegrationMethod(_integrationMethod);
    }
    
    json_t* oversamplingJ = json_object_get(rootJ, "oversampling");
    if (oversamplingJ && (_oversampling != json_integer_value(oversamplingJ))) {
      _oversampling = json_integer_value(oversamplingJ);

      // set new oversampling factor
      for(int ii = 0; ii < 16; ii++)
	svf[ii].SetFilterOversamplingFactor(_oversampling);
    }
    
    json_t* decimatorOrderJ = json_object_get(rootJ, "decimatorOrder");
    if (decimatorOrderJ && (_decimatorOrder != json_integer_value(decimatorOrderJ))) {
      _decimatorOrder = json_integer_value(decimatorOrderJ);

      // set new decimator order
      for(int ii = 0; ii < 16; ii++)
	svf[ii].SetFilterDecimatorOrder(_decimatorOrder);
    }
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
    
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(8.84, 13.64)), module, SVF_1::FREQ_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(11.24, 33.86)), module, SVF_1::RESO_PARAM));
    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(4.93, 84.38)), module, SVF_1::GAIN_PARAM));
    
    addParam(createParam<Trimpot>(mm2px(Vec(5.86, 51.52)), module, SVF_1::LINCV_ATTEN_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(18.621, 51.52)), module, SVF_1::EXPCV_ATTEN_PARAM));
    
    addParam(createParam<CKSSThree>(Vec(58.48, 248.3), module, SVF_1::MODE_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 65.52)), module, SVF_1::LINCV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 65.52)), module, SVF_1::EXPCV_INPUT));
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
      for(int ii = 0; ii < 16; ii++){    
	_module->svf[ii].SetFilterOversamplingFactor(_module->_oversampling);
      }
    }

    void step() override {
      MenuItem::step();
      rightText = _module->_oversampling == _oversampling ? "✔" : "";
    }
  };
  
  struct DecimatorOrderMenuItem : MenuItem {
    SVF_1* _module;
    const int _decimatorOrder;

    DecimatorOrderMenuItem(SVF_1* module, const char* label, int decimatorOrder)
      : _module(module)
      , _decimatorOrder(decimatorOrder)
    {
      this->text = label;
    }

    void onAction(const event::Action& e) override {
      _module->_decimatorOrder = _decimatorOrder;
      for(int ii = 0; ii < 16; ii++){    
	_module->svf[ii].SetFilterDecimatorOrder(_module->_decimatorOrder);
      }
    }

    void step() override {
      MenuItem::step();
      rightText = _module->_decimatorOrder == _decimatorOrder ? "✔" : "";
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
      for(int ii = 0; ii < 16; ii++){
	_module->svf[ii].SetFilterIntegrationMethod(_module->_integrationMethod);
      }
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
    menu->addChild(createMenuLabel("Decimator order"));
    menu->addChild(new DecimatorOrderMenuItem(a, "Decimator order: 8", 8));
    menu->addChild(new DecimatorOrderMenuItem(a, "Decimator order: 16", 16));
    menu->addChild(new DecimatorOrderMenuItem(a, "Decimator order: 32", 32));
    
    menu->addChild(new MenuEntry());
    menu->addChild(createMenuLabel("Integration Method"));
    menu->addChild(new IntegrationMenuItem(a, "Trapezoidal", SVF_TRAPEZOIDAL));
    menu->addChild(new IntegrationMenuItem(a, "Inverse Trapezoidal", SVF_INV_TRAPEZOIDAL));
  }
};

Model* modelSVF_1 = createModel<SVF_1, SVFWidget>("SVF-1");
