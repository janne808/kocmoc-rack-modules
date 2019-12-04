#include <cmath>
#include "plugin.hpp"
#include "phasor.h"

struct OP : Module {
  enum ParamIds {
    INDEX_PARAM,
    RATIO_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    PHASE_MOD_INPUT,
    RATIO_MOD_INPUT,
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
    configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "Modulation index");
    configParam(RATIO_PARAM, -12.f, 48.f, 0.f, "Ratio");
  }

  // create phasor instance
  Phasor *phasor = new Phasor((double)(0.0), (double)(440.0), (double)(APP->engine->getSampleRate()));
  
  void process(const ProcessArgs& args) override {
    // parameters
    int ratio = int(params[RATIO_PARAM].getValue());
    float index = params[INDEX_PARAM].getValue();

    // inputs
    float cv = inputs[CV_INPUT].getVoltage();
    float phase_mod = inputs[PHASE_MOD_INPUT].getVoltage();
    float ratio_mod = inputs[RATIO_MOD_INPUT].getVoltage();

    // apply ratio to cv
    cv += (float)(ratio+(int)(12.0*ratio_mod))/12.f;

    // clip negative cv
    if(cv < 0.0) {
      cv = 0.0;
    }

    // shape index parameter
    index *= index*index*index;
    
    // set operator frequency
    phasor->SetFrequency((double)((440.0/32.0) * std::pow(2.f, cv)));

    // set operator phase modulation
    phasor->SetPhaseModulation((double)(64.0*index*phase_mod));
    
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
    
    addParam(createParam<Trimpot>(mm2px(Vec(7.02, 17.703)), module, OP::INDEX_PARAM));
    addParam(createParam<RoundBlackKnob>(mm2px(Vec(4.94, 65.629)), module, OP::RATIO_PARAM));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 34.72)), module, OP::PHASE_MOD_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 55.465)), module, OP::RATIO_MOD_INPUT));    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.281, 87.548)), module, OP::CV_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.281, 103.3)), module, OP::OUTPUT_OUTPUT));
  }
};

Model* modelOP = createModel<OP, OPWidget>("OP");
