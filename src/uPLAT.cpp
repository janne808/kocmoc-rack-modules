/*
 *  (C) 2026 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#include "NfxReverb.hpp"

#include "fastmath.h"

struct uPLAT : Module {
  enum ParamIds {
    DECAY_PARAM,
    SIZE_PARAM,
    MOD_PARAM,
    DAMP_PARAM,
    PREDELAY_PARAM,
    BANDWIDTH_PARAM,
    GAIN_PARAM,
    DRYWET_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    INPUT_INPUT,
    CV_SIZE_INPUT,
    NUM_INPUTS
  };
  enum OutputIds {
    LEFT_OUTPUT,
    RIGHT_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    NUM_LIGHTS
  };

  // system samplerate
  float sampleRate;

  // reverb instance
  nfx::NfxReverb reverb;
  
  uPLAT() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Decay time");
    configParam(SIZE_PARAM, 0.f, 1.f, 1.f, "Tank size");
    configParam(MOD_PARAM, 0.f, 1.f, 0.5f, "Tank modulation amount");
    configParam(DAMP_PARAM, 0.f, 1.f, 0.8f, "Tank damping amount");
    configParam(PREDELAY_PARAM, 0.f, 1.f, 0.f, "Predelay time");
    configParam(BANDWIDTH_PARAM, 0.f, 1.f, 0.5f, "Tank bandwidth");
    configParam(GAIN_PARAM, 0.f, 2.f, 1.f, "Gain");
    configParam(DRYWET_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet mix");
    configInput(CV_SIZE_INPUT, "Size CV");
    configInput(INPUT_INPUT, "Audio");
    configOutput(LEFT_OUTPUT, "Reverb Left");
    configOutput(RIGHT_OUTPUT, "Reverb Right");
    configBypass(INPUT_INPUT, LEFT_OUTPUT);
    configBypass(INPUT_INPUT, RIGHT_OUTPUT);

    // get system samplerate
    sampleRate = APP->engine->getSampleRate();

    // initialize reverb samplerate
    reverb.setSampleRate(sampleRate);
    reverb.setPredelay12Bit(false);
    reverb.setAntiAlias(true);
    reverb.setAntiImage(true);
    reverb.reset();
  }

  void process(const ProcessArgs& args) override {
    // parameter variables
    float decay = params[DECAY_PARAM].getValue();
    float size = params[SIZE_PARAM].getValue();
    float mod = params[MOD_PARAM].getValue();
    float damp = params[DAMP_PARAM].getValue();
    float predelay = params[PREDELAY_PARAM].getValue();
    float bandwidth = params[BANDWIDTH_PARAM].getValue();
    float gain = params[GAIN_PARAM].getValue();
    float drywet = params[DRYWET_PARAM].getValue();
    
    // inputs
    float cv_size = inputs[CV_SIZE_INPUT].getVoltageSum();

    // outputs
    float out_left, out_right;

    // process cv
    size += cv_size;
    if(size > 1.f) {
      size = 1.f;
    }
    else if(size < 0.f) {
      size = 0.f;
    }
    
    // set reverb parameters
    reverb.setDecay(decay);
    reverb.setSize(size);
    reverb.setMod(mod);
    reverb.setDamp(damp);
    
    reverb.setPredelay(predelay);
    reverb.setBandwidth(bandwidth);
    
    reverb.setGain(1.f);
    reverb.setMix(drywet);

    // process input
    reverb.processStereo(inputs[INPUT_INPUT].getVoltageSum() * gain, out_left, out_right);

    // set output
    outputs[LEFT_OUTPUT].setVoltage((float)(out_left));
    outputs[RIGHT_OUTPUT].setVoltage((float)(out_right));
  }
  
  void onSampleRateChange() override {
    // new system samplerate
    sampleRate = APP->engine->getSampleRate();

    // set reverb samplerate
    reverb.setSampleRate(sampleRate);
    reverb.reset();
  }

  void onReset() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();

    // set reverb samplerate
    reverb.setSampleRate(sampleRate);
    reverb.reset();
  }

  void onAdd() override {
    // get system samplerate
    sampleRate = APP->engine->getSampleRate();

    // set reverb samplerate
    reverb.setSampleRate(sampleRate);
    reverb.reset();
  }  
};

struct uPLATWidget : ModuleWidget {
  uPLATWidget(uPLAT* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/uPLAT.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(17.78 - 8.2, 21.64)), module, uPLAT::DECAY_PARAM));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(17.78 - 8.2, 21.64 + 20.0)), module, uPLAT::SIZE_PARAM)); 
    
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(17.78 + 8.2, 21.64)), module, uPLAT::DAMP_PARAM));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(17.78 + 8.2, 21.64 + 20.0)), module, uPLAT::PREDELAY_PARAM));
    
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(17.78 - 8.2, 60.64)), module, uPLAT::MOD_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(17.78 + 8.2, 60.64)), module, uPLAT::BANDWIDTH_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(17.78 - 8.2, 104.7 - 11.0)), module, uPLAT::GAIN_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(17.78 + 8.2, 104.7 - 27.5)), module, uPLAT::DRYWET_PARAM));
       
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.78 - 8.2, 104.7 - 27.5)), module, uPLAT::CV_SIZE_INPUT));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.78 - 8.2, 104.7)), module, uPLAT::INPUT_INPUT));
    
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.78 + 8.2, 104.7 - 11.0)), module, uPLAT::LEFT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.78 + 8.2, 104.7)), module, uPLAT::RIGHT_OUTPUT));
  }
};

Model* modeluPLAT = createModel<uPLAT, uPLATWidget>("uPLAT");
