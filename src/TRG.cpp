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
 *  along with Kocmoc VCV Rack plugin. If not, see <http://www.gnu.org/licenses/>.
 */

#include "plugin.hpp"

#define MAX_STEPS 32

// define grid geometry
#define GRID_X_OFFSET 10
#define GRID_Y_OFFSET 6
#define GRID_STEP_WIDTH 20
#define GRID_STEP_HEIGHT 20
#define GRID_STEP_X_MARGIN 10
#define GRID_STEP_Y_MARGIN 4
#define GRID_ACTIVE_STEP_RADIUS 2.5
#define GRID_PAGE_TOGGLE_HEIGHT 6
#define GRID_PAGE_TOGGLE_Y_MARGIN 2

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
    configParam(LEN_PARAM, 1.f, 32.f, 32.f, "Seq length");
    configInput(CLK_INPUT, "Clock");
    configInput(RST_INPUT, "Reset");
    configOutput(GATE_OUTPUT, "Gate");
    configBypass(CLK_INPUT, GATE_OUTPUT);
    
    // reset current step
    step = 0;
    
    // reset steps
    for(int ii = 0; ii < MAX_STEPS; ii++){
      steps[ii] = 0;
    }

    // reset sequence length
    seq_length = MAX_STEPS;
  }

  float displayWidth = 0, displayHeight = 0;
  int steps[MAX_STEPS];
  int step;
  int clock_state = 0;
  int reset_state = 0;
  float gate_state = 0.f;
  int seq_length;
  int page = 0;
  
  // menu variables
  int _followactivestep = 1;
  
  void process(const ProcessArgs& args) override {
    // switch clock state
    if(clock_state == 0 && inputs[CLK_INPUT].getVoltage() > 0.5f){
      clock_state = 1;
      step += 1;
      
      if(step > seq_length - 1){
	step = 0;
      }

      if(steps[step] == 1){
	gate_state = 1.f;
      }
      else{
	gate_state = 0.f;
      }
      
      // compute ui page
      if(_followactivestep){
	updatePage();
      }
    }
    else if(clock_state == 1 && inputs[CLK_INPUT].getVoltage() < 0.5f){
      clock_state = 0;
      gate_state = 0.f;
    }

    // switch reset state
    if(reset_state == 0 && inputs[RST_INPUT].getVoltage() > 0.5f){
      reset_state = 1;

      // set clock state if reset
      clock_state = 1;
      step = 0;
      if(steps[step] == 1){
	gate_state = 1.f;
      }
      else{
	gate_state = 0.f;
      }

      // compute ui page
      if(_followactivestep){
	updatePage();
      }
    }
    else if(reset_state == 1 && inputs[RST_INPUT].getVoltage() < 0.5f){
      reset_state = 0;
    }
    
    // output gate
    outputs[GATE_OUTPUT].setVoltage(gate_state * 10.f);

    // get sequence length
    seq_length = (int)(params[LEN_PARAM].getValue());    
  }

  void onRandomize() override {
    // randomize steps
    for(int ii = 0; ii < MAX_STEPS; ii++){
      steps[ii] = (rack::random::uniform() > 0.5) ? 1 : 0;
    }
  }
  
  void onReset() override {
    // reset clock and gate state
    clock_state = 0;
    reset_state = 0;
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
    json_object_set_new(rootJ, "page", json_integer(page));

    json_object_set_new(rootJ, "followactivestep", json_integer(_followactivestep));
    
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

    json_t *pageJ = json_object_get(rootJ, "page");
    if(pageJ){
      page = (int)(json_integer_value(pageJ));
    }
    
    json_t *followactivestepJ = json_object_get(rootJ, "followactivestep");
    if(followactivestepJ){
      _followactivestep = (int)(json_integer_value(followactivestepJ));
    }    
  }

  bool isClickOnStep(float x, float y){
    if(( (x > GRID_X_OFFSET && x < GRID_X_OFFSET + GRID_STEP_WIDTH) ||
	 (x > GRID_X_OFFSET + GRID_STEP_WIDTH + GRID_STEP_X_MARGIN &&
	  x < GRID_X_OFFSET + 2*GRID_STEP_WIDTH + GRID_STEP_X_MARGIN ) ) &&
	 y > GRID_Y_OFFSET && y < GRID_Y_OFFSET + (MAX_STEPS / 4) * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN)){
      return true;
    }
    
    return false;
  }

  bool isClickOnPageSelect(float x, float y){
    if(( (x > GRID_X_OFFSET && x < GRID_X_OFFSET + 2*GRID_STEP_WIDTH + GRID_STEP_X_MARGIN) ) &&
         y > GRID_Y_OFFSET + (MAX_STEPS / 4) * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN) &&
         y < GRID_Y_OFFSET + (MAX_STEPS / 4) * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN) + 2 + 12){
      return true;
    }
    
    return false;
  }

  void updatePage(){
    page = step / (MAX_STEPS / 2);
  }
};

struct TRGDisplay : Widget {
  float dragX = 0;
  float dragY = 0;
  float initX = 0;
  float initY = 0;
  int currentStep = 0;
  int currentClickState = 0;
  TRG *module = NULL;
  TRGDisplay(){}

  void onButton(const event::Button &e) override {
    if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
      e.consume(this);

      // store initial click coords
      initX = e.pos.x;
      initY = e.pos.y;
      
      // is click on a step button
      if(module->isClickOnStep(e.pos.x, e.pos.y)){
	// compute step number
	int nn = (int)((e.pos.y - GRID_Y_OFFSET) / (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN));
	if(e.pos.x > GRID_X_OFFSET + GRID_STEP_WIDTH + GRID_STEP_X_MARGIN &&
	   e.pos.x < GRID_X_OFFSET + 2*GRID_STEP_WIDTH + GRID_STEP_X_MARGIN ){
	  nn += 8;
	}
	// add in page
	nn += module->page * (MAX_STEPS / 2);
  	module->steps[nn] = !module->steps[nn];
	currentStep = nn;
	currentClickState = module->steps[currentStep]; 
      }
      // is click on page select button
      else if(module->isClickOnPageSelect(e.pos.x, e.pos.y)){
	if(!module->_followactivestep){
	  module->page = !module->page;
	}
      }
    }
  }

  void onDragStart(const event::DragStart &e) override {
    dragX = APP->scene->rack->getMousePos().x;
    dragY = APP->scene->rack->getMousePos().y;    
  }

  void onDragMove(const event::DragMove &e) override {
    float newDragX = APP->scene->rack->getMousePos().x;
    float newDragY = APP->scene->rack->getMousePos().y;
    float currentX = initX+(newDragX-dragX);
    float currentY = initY+(newDragY-dragY);
    
    // is drag on a step button
    if(module->isClickOnStep(currentX, currentY)){
      // compute step number
      int nn = (int)((currentY - GRID_Y_OFFSET) / (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN));
      if(currentX > GRID_X_OFFSET + GRID_STEP_WIDTH + GRID_STEP_X_MARGIN &&
	 currentX < GRID_X_OFFSET + 2*GRID_STEP_WIDTH + GRID_STEP_X_MARGIN ){
	nn += 8;
      }
      
      // add in page
      nn += module->page * (MAX_STEPS / 2);

      // switch state just once
      if(nn != currentStep) {
	module->steps[nn] = currentClickState;
	currentStep = nn;
      }
    }
  }

  void drawSequenceGrid(const DrawArgs &args, int moduleStep, int modulePage, int moduleSeqLength, int *moduleSteps) {
    // draw sequence grid
    for(int ii = 0; ii < MAX_STEPS / 2; ii++){
      int xx = ii / (MAX_STEPS / 4);
      int yy = ii % (MAX_STEPS / 4);
      int page = modulePage;
      NVGcolor step_color;
      int current_step = ii + page * (MAX_STEPS / 2);

      // draw active step in bright color
      if(current_step < moduleSeqLength){
	step_color = nvgRGB(252, 252, 3);
      }
      else{
	step_color = nvgRGB(62, 62, 0);
      }
      
      nvgStrokeColor(args.vg, step_color);
      nvgFillColor(args.vg, step_color);
      nvgStrokeWidth(args.vg, 1);
      nvgBeginPath(args.vg);
      nvgRect(args.vg, GRID_X_OFFSET + xx * (GRID_STEP_WIDTH + GRID_STEP_X_MARGIN),
	      GRID_Y_OFFSET + yy * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN), GRID_STEP_WIDTH, GRID_STEP_HEIGHT);
      
      // render step based on its state
      if(moduleSteps[current_step] == 1){
	nvgFill(args.vg);
      }
      else{
	nvgStroke(args.vg);
      }
      
      // render current step
      if(current_step == moduleStep){
	// determine color from step state
	if(moduleSteps[current_step] == 1 ){
	  nvgFillColor(args.vg, nvgRGB(20, 30, 33));
	}
	else {
	  nvgFillColor(args.vg, step_color);
	}
	nvgBeginPath(args.vg);
	nvgCircle(args.vg, GRID_X_OFFSET + GRID_STEP_WIDTH/2.f + xx * (GRID_STEP_WIDTH + GRID_STEP_X_MARGIN),
		  GRID_Y_OFFSET + GRID_STEP_HEIGHT/2.f + (moduleStep % (MAX_STEPS / 4)) * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN), GRID_ACTIVE_STEP_RADIUS);
	nvgFill(args.vg);
      }

      // render current page
      nvgFillColor(args.vg, nvgRGB(252, 252, 3));
      nvgBeginPath(args.vg);
      nvgRect(args.vg, GRID_X_OFFSET + page * (GRID_STEP_WIDTH + GRID_STEP_X_MARGIN),
	      GRID_Y_OFFSET + GRID_PAGE_TOGGLE_Y_MARGIN + (MAX_STEPS / 4) * (GRID_STEP_HEIGHT + GRID_STEP_Y_MARGIN), GRID_STEP_WIDTH, GRID_PAGE_TOGGLE_HEIGHT);
      nvgFill(args.vg);
    }      
  }

  void drawLayer(const DrawArgs &args, int layer) override {
    if(!module)
      return;
    
    if(layer == 1) {
      // draw sequence grid
      drawSequenceGrid(args, module->step, module->page, module->seq_length, module->steps);
    }
    
    Widget::drawLayer(args, layer);
  }
  
  void draw(const DrawArgs &args) override {
    // background
    nvgFillColor(args.vg, nvgRGB(20, 30, 33));
    nvgBeginPath(args.vg);
    nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
    nvgFill(args.vg);

    // draw default grid if on module browser
    if(module == NULL){
      int moduleStep = 0;
      int modulePage = 0;
      int moduleSeqLength = 32;
      int moduleSteps[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

      // draw default sequence grid
      drawSequenceGrid(args, moduleStep, modulePage, moduleSeqLength, moduleSteps);
    }
  }
};

struct TRGWidget : ModuleWidget {
  TRGWidget(TRG* module) {
    setModule(module);
    setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/TRG.svg")));

    TRGDisplay *display = new TRGDisplay();
    display->module = module;
    display->box.pos = Vec(10, 78);
    display->box.size = Vec(70, 10 + (MAX_STEPS / 4) * (20 + 4) + 10);
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

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.96, 20.12)), module, TRG::CLK_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(21.48, 20.12)), module, TRG::RST_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.96, 110.68)), module, TRG::GATE_OUTPUT));
  }

  struct TRGMenuItem : MenuItem {
    TRG* _module;
    const int _followactivestep;

    TRGMenuItem(TRG* module, const char* label, int followactivestep)
      : _module(module)
      , _followactivestep(followactivestep)
    {
      this->text = label;
    }

    void onAction(const event::Action& e) override {
      // toggle action
      if(_module->_followactivestep){
	_module->_followactivestep = 0;
      }else{
	_module->_followactivestep = 1;
	_module->updatePage();
      }
    }

    void step() override {
      MenuItem::step();
      rightText = _module->_followactivestep == _followactivestep ? "✔" : "";
    }
  };
  
  void appendContextMenu(Menu* menu) override {
    TRG* a = dynamic_cast<TRG*>(module);
    assert(a);
    
    menu->addChild(new MenuEntry());
    menu->addChild(new TRGMenuItem(a, "Follow active step", 1));
  }  
};

Model* modelTRG = createModel<TRG, TRGWidget>("TRG");
