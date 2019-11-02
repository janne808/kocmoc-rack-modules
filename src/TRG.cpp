#include "plugin.hpp"

#define MAX_STEPS 16

struct TRG : Module {
  enum ParamIds {
    LEN_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    CLK_INPUT,
    RST_INPUT,
    NUM_INPUTS
  };
  enum OutputIds {
    GATE_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    NUM_LIGHTS
  };

  TRG() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(LEN_PARAM, 1.f, 16.f, 16.f, "Seq length");
    
    // reset current step
    step = 0;
    
    // reset steps
    for(int ii = 0; ii < MAX_STEPS; ii++){
      steps[ii] = 0;
    }

    // reset sequence length
    seq_length = 16;

    // reset reset latch
    reset_latch = 0;
  }

  float displayWidth = 0, displayHeight = 0;
  int steps[MAX_STEPS];
  int step;
  int clock_state = 0;
  int reset_state = 0;
  int reset_latch = 0;
  float gate_state = 0.f;
  int seq_length;
  
  void process(const ProcessArgs& args) override {
    // switch clock state
    if(clock_state == 0 && inputs[CLK_INPUT].getVoltage() > 0.5f){
      clock_state = 1;
      step += 1;
      if(step > seq_length - 1 || reset_latch){
	step = 0;
	reset_latch = 0;
      }
      if(steps[step] == 1){
	gate_state = 1.f;
      }
      else{
	gate_state = 0.f;
      }
    }
    else if(clock_state == 1 && inputs[CLK_INPUT].getVoltage() < 0.5f){
      clock_state = 0;
      gate_state = 0.f;
    }

    // switch reset state
    if(reset_state == 0 && inputs[RST_INPUT].getVoltage() > 0.5f){
      reset_state = 1;
      reset_latch = 1;
    }
    else if(reset_state == 1 && inputs[RST_INPUT].getVoltage() < 0.5f){
      reset_state = 0;
    }
    
    // output gate
    outputs[GATE_OUTPUT].setVoltage(gate_state * 10.f);

    // get sequence length
    seq_length = (int)(params[LEN_PARAM].getValue());    
  }

  void onReset() override {
    // reset clock and gate state
    clock_state = 0;
    reset_state = 0;
    reset_latch = 0;
    gate_state = 0.f;

    // reset current step
    step = 0;
    
    // reset steps
    for(int ii = 0; ii < MAX_STEPS; ii++){
      steps[ii] = 0;
    }
  }

  json_t* dataToJson() override {
    json_t *rootJ = json_object();

    json_t *stepsJ = json_array();
    for (int ii = 0; ii < MAX_STEPS; ii++){
      json_array_append_new(stepsJ, json_integer(steps[ii]));
    }
    json_object_set_new(rootJ, "steps", stepsJ);
    
    json_object_set_new(rootJ, "step", json_integer(step));

    return rootJ;
  }

  void dataFromJson(json_t *rootJ) override {
    json_t *stepsJ = json_object_get(rootJ, "steps");
    if(stepsJ){
      for (int ii = 0; ii < MAX_STEPS; ii++){
	steps[ii] = (int)(json_integer_value(json_array_get(stepsJ, ii)));
      }
    }
    
    json_t *stepJ = json_object_get(rootJ, "step");
    if(stepJ){
      step = (int)(json_integer_value(stepJ));
    }
  }
};

struct TRGDisplay : Widget {
  TRG *module;
  TRGDisplay(){}

  void onButton(const event::Button &e) override {
    if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
      e.consume(this);

      // is click on a step button
      if(( (e.pos.x > 10 && e.pos.x < 30) || (e.pos.x > 40 && e.pos.x < 60) ) &&
	 e.pos.y > 10 && e.pos.y < 10 + (MAX_STEPS / 2) * (20 + 4)){
	// compute step number
	int nn = (int)((e.pos.y - 10.f) / 24.f);
	if(e.pos.x > 40 && e.pos.x < 60){
	  nn += 8;
	}
  	module->steps[nn] = !module->steps[nn];
      }
    }
  }

  void draw(const DrawArgs &args) override {
    //background
    nvgFillColor(args.vg, nvgRGB(20, 30, 33));
    nvgBeginPath(args.vg);
    nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
    nvgFill(args.vg);
    
    if(module == NULL) return;
    
    // draw sequence grid
    for(int ii = 0; ii < MAX_STEPS; ii++){
      int xx = ii / (MAX_STEPS / 2);
      int yy = ii % (MAX_STEPS / 2);
      NVGcolor step_color;

      // draw active step in bright color
      if(ii < module->seq_length){
	step_color = nvgRGB(252, 252, 3);
      }
      else{
	step_color = nvgRGB(62, 62, 0);
      }
      
      nvgStrokeColor(args.vg, step_color);
      nvgFillColor(args.vg, step_color);
      nvgBeginPath(args.vg);
      nvgRect(args.vg, 10 + xx * (20 + 10),
	      10 + yy * (20 + 4), 20, 20);
      
      // render step based on its state
      if(module->steps[ii] == 1){
	nvgFill(args.vg);
      }
      else{
	nvgStroke(args.vg);
      }

      // render current step
      if(ii == module->step){
	// determine color from step state
	if(module->steps[ii] == 1 ){
	  nvgFillColor(args.vg, nvgRGB(20, 30, 33));
	}
	else {
	  nvgFillColor(args.vg, step_color);
	}
	nvgBeginPath(args.vg);
	nvgCircle(args.vg, 10.f + 10.f + xx * (20 + 10),
		  10.f + 10.f + (module->step % (MAX_STEPS / 2)) * (20.f + 4.f), 2.5f);
	nvgFill(args.vg);
      }
    }
  }
};

struct TRGWidget : ModuleWidget {
  TRGWidget(TRG* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/TRG.svg")));

    TRGDisplay *display = new TRGDisplay();
    display->module = module;
    display->box.pos = Vec(10, 82);
    display->box.size = Vec(70, 10 + (MAX_STEPS / 2) * (20 + 4) + 10);
    addChild(display);
    if(module != NULL){
      module->displayWidth = display->box.size.x;
      module->displayHeight = display->box.size.y;
    }
		
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParam<RoundBlackSnapKnob>(mm2px(Vec(16.8, 105.6)), module, TRG::LEN_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 21.92)), module, TRG::CLK_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.88, 21.92)), module, TRG::RST_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.96, 110.68)), module, TRG::GATE_OUTPUT));
  }
};


Model* modelTRG = createModel<TRG, TRGWidget>("TRG");
