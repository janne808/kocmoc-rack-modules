/*
 *  (C) 2021 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#define DDLY_MAX_DELAY_TIME 3
#define DDLY_TIME_THRESHOLD 0.006
#define DDLY_CLK_TIME_THRESHOLD 0.00002
#define DDLY_FADE_RATE 0.02

struct DDLY : Module {
  enum ParamIds {
    TIME_PARAM,
    FB_PARAM,
    TIME_CV_ATTEN_PARAM,
    FB_CV_ATTEN_PARAM,
    DRY_WET_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    TIME_CV_INPUT,
    FB_CV_INPUT,
    CLK_INPUT,
    RETURN_INPUT,
    INPUT_INPUT,
    NUM_INPUTS
  };
  enum OutputIds {
    SEND_OUTPUT,
    OUTPUT_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    NUM_LIGHTS
  };

  int sampleRate;
  
  // ring buffer variables
  float *ringBuffer;
  int bufferLength;
  int writePointer;

  // delay variables
  float time2;
  
  // crossfader variables
  int fade_state;
  float fade_value;
  float fade0_time, fade1_time;

  // clock variables
  float last_clk;
  int clk_counter;
  int clk_period;
  int clk_n;

  // dc blocking highpass filter variables
  float hp, hp2;
  
  DDLY() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(TIME_PARAM, 0.f, 1.f, 0.5f, "Delay time");
    configParam(FB_PARAM, 0.f, 1.f, 0.f, "Feedback");
    configParam(TIME_CV_ATTEN_PARAM, -1.f, 1.f, 0.f, "CV Amount");
    configParam(FB_CV_ATTEN_PARAM, -1.f, 1.f, 0.f, "CV Amount");
    configParam(DRY_WET_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet");
    configInput(TIME_CV_INPUT, "Time CV");
    configInput(FB_CV_INPUT, "Feedback CV");
    configInput(CLK_INPUT, "Clock");
    configOutput(SEND_OUTPUT, "Send");
    configInput(RETURN_INPUT, "Return");
    configInput(INPUT_INPUT, "Input");
    configOutput(OUTPUT_OUTPUT, "Delay");
    configBypass(INPUT_INPUT, OUTPUT_OUTPUT);

    // get samplerate
    sampleRate = APP->engine->getSampleRate();
    
    // init ringbuffer
    bufferLength = DDLY_MAX_DELAY_TIME*sampleRate;
    writePointer = 0;
    ringBuffer = new float[bufferLength];

    for(int ii=0; ii<bufferLength; ii++){
      ringBuffer[ii] = 0.f;
    }

    time2 = 0.f;

    fade_state = 0;
    fade0_time = fade1_time = 0.f;

    // init clock detector
    last_clk = 0.f;
    clk_period = 0;
    clk_counter = 0;
    clk_n = 0;

    // init highpass filter
    hp = hp2 = 0.f;
  }

  void process(const ProcessArgs& args) override {
    float time = params[TIME_PARAM].getValue();
    float feedback = params[FB_PARAM].getValue();
    float drywet = params[DRY_WET_PARAM].getValue();

    float time_cv_atten = params[TIME_CV_ATTEN_PARAM].getValue();
    float fb_cv_atten = params[FB_CV_ATTEN_PARAM].getValue();

    float time_cv = inputs[TIME_CV_INPUT].getVoltage();
    float fb_cv = inputs[FB_CV_INPUT].getVoltage();
    
    float ret = inputs[RETURN_INPUT].getVoltage();
    float clk = inputs[CLK_INPUT].getVoltage();
    
    float input = inputs[INPUT_INPUT].getVoltage();
    float delay;

    // dc blocking filter for input
    float hp_input = input;
    hp += 0.0005f*(hp_input - hp);
    input = hp - hp_input;
    
    // sum in time modulation control voltage
    // note that time is a float normalized to buffer length
    time += time_cv_atten*(time_cv/5.f);

    // clip time value
    if(time > 0.9985f){
      time = 0.9985f;
    }
    
    // sum in feedback modulation control voltage
    feedback += fb_cv_atten*(fb_cv/5.f);

    // clip feedback value
    if(feedback < 0.f){
      feedback = 0.f;
    }
    else if(feedback > 1.f){
      feedback = 1.f;
    }

    // check for clock signal input
    if(inputs[CLK_INPUT].isConnected()){
      // detect rising clock edge
      if(last_clk <= 0.0f && clk > 0.0f){
	clk_period = clk_counter;
	
	clk_n++;
	if(clk_n > 7){
	  clk_n = 2;
	}
	
	clk_counter = 0;
      }

      // count for clock period in samples
      clk_counter++;

      // only engage in clock sync after two detected edges
      if(clk_period > 0 && clk_n > 1){
	// clock rate division and multiplier table
	const float div_table[16] = { 0.125f, 0.25f, 0.375f, 0.5f,
	                              0.625f, 0.75f, 0.875f, 1.f,
				      1.125f, 1.25f, 1.375f, 1.5f,
				      1.625f, 1.75f, 1.875f, 2.f
	                            };
	
	float clk_time;
	float ratio;

	// tighten clock divisions
	if(time < 0.5f){
	  ratio = div_table[static_cast <int> (15.f*time)];
	  ratio = ratio*ratio;
	}
	else{
	  ratio = div_table[static_cast <int> (15.f*time)];
	}
	
	clk_time = (static_cast <float> (clk_period))/(static_cast <float> (sampleRate));
	time = ratio*clk_time/(static_cast <float> (DDLY_MAX_DELAY_TIME));
	
	// clip time value
	// minimum delay time
	if(time < 0.0005f){
	  time = 0.0005f;
	}	
	else if(time > 0.9985f){
	  time = 0.9985f;
	}
    
	// add hysteresis threshold to time parameter value
	// for noisy real world cv input
	if(abs(time-time2) > DDLY_CLK_TIME_THRESHOLD){
	  time2 = time;
	  
	  // trigger crossfade
	  if(fade_state){
	    fade_state = 0;
	    fade0_time = time2;
	  }
	  else{
	    fade_state = 1;
	    fade1_time = time2;	
	  }
	}
      }
      else{
	// this branch is reached if clock signal input is connected
	// but edge detection hasn't received two clock pulses yet
	
	// add hysteresis threshold to time parameter value
	// for noisy real world cv input
	if(abs(time-time2) > DDLY_TIME_THRESHOLD){
	  time2 = time;
	
	  // trigger crossfade
	  if(fade_state){
	    fade_state = 0;
	    fade0_time = time2*time2*time2;
	    if(fade0_time < 0.0004f){
	      fade0_time = 0.0004f;
	    }
	  }
	  else{
	    fade_state = 1;
	    fade1_time = time2*time2*time2;	
	    if(fade1_time < 0.0004f){
	      fade1_time = 0.0004f;
	    }
	  }
	}
      }
    }
    else{
      // clock signal is not connected
      
      // disable clock counter
      clk_counter = 0;
      clk_period = 0;
      clk_n = 0;
	
      // add hysteresis threshold to time parameter value
      // for noisy real world cv input
      if(abs(time-time2) > DDLY_TIME_THRESHOLD){
	time2 = time;

	// trigger crossfade
	if(fade_state){
	  fade_state = 0;
	  fade0_time = time2*time2*time2;
	  if(fade0_time < 0.0004f){
	    fade0_time = 0.0004f;
	  }
	}
	else{
	  fade_state = 1;
	  fade1_time = time2*time2*time2;	
	  if(fade1_time < 0.0004f){
	    fade1_time = 0.0004f;
	  }
	}
      }
    }

    // update crossfade
    if(fade_state){
      fade_value += DDLY_FADE_RATE;
      if(fade_value > 1.f){
	fade_value = 1.f;
      }
    }
    else{
      fade_value -= DDLY_FADE_RATE;
      if(fade_value < 0.f){
	fade_value = 0.f;
      }
    }
      
    // read delayed signal
    delay = (1.f - fade_value)*readDelay(fade0_time) + fade_value*readDelay(fade1_time);

    // update buffer
    // with dc blocking highpass filter
    if(inputs[RETURN_INPUT].isConnected()){
      writeDelay(ret);    
    }
    else{
      writeDelay(input + feedback*delay);    
    }
    
    // set send output
    outputs[SEND_OUTPUT].setVoltage(input + feedback*delay);

    // dc blocking filter for output
    float hp_output = (1.f - drywet)*input + drywet*delay;
    hp2 += 0.0005f*(hp_output - hp2);
    
    // set output
    outputs[OUTPUT_OUTPUT].setVoltage(hp2 - hp_output);
    
    // save last clk value for edge detection
    last_clk = clk;
  }

  float readDelay(float time){
    int readPointer, readPointer2;
    float frac;
    
    readPointer = writePointer - (int)(time*bufferLength);
    
    if(readPointer < 0){
      readPointer += bufferLength;
    }

    readPointer2 = readPointer - 1;
    
    if(readPointer2 < 0){
      readPointer2 += bufferLength;
    }

    // simple fractional linear interpolation
    frac = (float)(time*bufferLength) - (float)((int)(time*bufferLength));
    return (1.f - frac)*ringBuffer[readPointer] + (frac)*ringBuffer[readPointer2];
  }

  void writeDelay(float input){
      writePointer += 1;
      if(writePointer > bufferLength - 1){
	writePointer -= bufferLength;
      }

      ringBuffer[writePointer] = input;
  }
  
  void onAdd() override {
    // reallocate ringbuffer
    if(ringBuffer){
      delete ringBuffer;
    }

    sampleRate = APP->engine->getSampleRate();
    
    // init ringbuffer
    bufferLength = DDLY_MAX_DELAY_TIME*sampleRate;
    writePointer = 0;
    ringBuffer = new float[bufferLength];

    for(int ii=0; ii<bufferLength; ii++){
      ringBuffer[ii] = 0.f;
    }

    // init crossfade state
    fade_state = 0;
    fade0_time = fade1_time = 0.f;

    // init highpass filter
    hp = hp2 = 0.f;
  }
  
  void onReset() override {
    // reallocate ringbuffer
    if(ringBuffer){
      delete ringBuffer;
    }

    sampleRate = APP->engine->getSampleRate();
    
    // init ringbuffer
    bufferLength = DDLY_MAX_DELAY_TIME*sampleRate;
    writePointer = 0;
    ringBuffer = new float[bufferLength];

    for(int ii=0; ii<bufferLength; ii++){
      ringBuffer[ii] = 0.f;
    }

    // init crossfade state
    fade_state = 0;
    fade0_time = fade1_time = 0.f;

    // init highpass filter
    hp = hp2 = 0.f;
  }
  
  void onSampleRateChange() override {
    // reallocate ringbuffer
    if(ringBuffer){
      delete ringBuffer;
    }

    sampleRate = APP->engine->getSampleRate();
    
    // init ringbuffer
    bufferLength = DDLY_MAX_DELAY_TIME*sampleRate;
    writePointer = 0;
    ringBuffer = new float[bufferLength];

    for(int ii=0; ii<bufferLength; ii++){
      ringBuffer[ii] = 0.f;
    }

    // init crossfade state
    fade_state = 0;
    fade0_time = fade1_time = 0.f;

    // init highpass filter
    hp = 0.f;
  }
};

struct DDLYWidget : ModuleWidget {
  DDLYWidget(DDLY* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/DDLY.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(15.2, 17.64)), module, DDLY::TIME_PARAM));
    addParam(createParam<RoundLargeBlackKnob>(mm2px(Vec(15.2, 42.86)), module, DDLY::FB_PARAM));
    
    addParam(createParam<Trimpot>(mm2px(Vec(4.46, 16.64)), module, DDLY::TIME_CV_ATTEN_PARAM));
    addParam(createParam<Trimpot>(mm2px(Vec(4.46, 41.82)), module, DDLY::FB_CV_ATTEN_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.45 , 28.32)), module, DDLY::TIME_CV_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.45, 53.5)), module, DDLY::FB_CV_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.96, 68.7)), module, DDLY::SEND_OUTPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.681, 68.7)), module, DDLY::RETURN_INPUT));
    
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 86.3)), module, DDLY::CLK_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 104.7)), module, DDLY::INPUT_INPUT));

    addParam(createParam<RoundSmallBlackKnob>(mm2px(Vec(17.96, 82.8)), module, DDLY::DRY_WET_PARAM));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(21.681, 104.7)), module, DDLY::OUTPUT_OUTPUT));
  }
};

Model* modelDDLY = createModel<DDLY, DDLYWidget>("DDLY");

