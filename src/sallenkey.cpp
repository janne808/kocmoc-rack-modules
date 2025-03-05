/*
 *  (C) 2025 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#include <cstdlib>
#include <cmath>
#include "sallenkey.h"

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 16

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.9

// maximum newton-raphson iteration steps
#define SKF_MAX_NEWTON_STEPS 8

// check for newton-raphson breaking limit
#define SKF_NEWTON_BREAKING_LIMIT 1

// constructor
SKFilter::SKFilter(double newCutoff, double newResonance, int newOversamplingFactor,
		   SKFilterMode newFilterMode, double newSampleRate,
		   SKIntegrationMethod newIntegrationMethod, int newDecimatorOrder){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;
  oversamplingFactor = newOversamplingFactor;
  decimatorOrder = newDecimatorOrder;

  SetFilterIntegrationRate();

  // initialize filter state
  p0 = p1 = out = 0.0f;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0f;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0f;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor,
		         IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0f,
		         decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor,
		       IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0,
		       decimatorOrder);
#endif
}

// default constructor
SKFilter::SKFilter(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  filterMode = SK_LOWPASS_MODE;
  sampleRate = 44100.0;
  oversamplingFactor = 2;
  decimatorOrder = IIR_DOWNSAMPLE_ORDER;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = out = 0.0f;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0f;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0f;
  
  integrationMethod = SK_TRAPEZOIDAL;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor,
		         IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0f,
		         decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor,
		       IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0,
		       decimatorOrder);
#endif
}

// default destructor
SKFilter::~SKFilter(){
  delete iir;
}

void SKFilter::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = out = 0.0f;

  // initialize filter inputs
  input_lp = input_bp = input_hp = 0.0f;
  input_lp_t1 = input_bp_t1 = input_hp_t1 = 0.0f;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);
  iir->SetFilterOrder(decimatorOrder);
}

void SKFilter::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void SKFilter::SetFilterMode(SKFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void SKFilter::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterIntegrationMethod(SKIntegrationMethod method){
  integrationMethod = method;
}

void SKFilter::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);
  iir->SetFilterOrder(decimatorOrder);

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterDecimatorOrder(int newDecimatorOrder){
  decimatorOrder = newDecimatorOrder;
  iir->SetFilterOrder(decimatorOrder);
}

void SKFilter::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 0.55f){
    dt = 0.55f;
  }
}

double SKFilter::GetFilterCutoff(){
  return cutoffFrequency;
}

double SKFilter::GetFilterResonance(){
  return Resonance;
}

int SKFilter::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

int SKFilter::GetFilterDecimatorOrder(){
  return decimatorOrder;
}

#ifdef FLOATDSP
float SKFilter::GetFilterOutput(){
  return out;
}
#else
double SKFilter::GetFilterOutput(){
  return out;
}
#endif

SKFilterMode SKFilter::GetFilterMode(){
  return filterMode;
}

double SKFilter::GetFilterSampleRate(){
  return sampleRate;
}

SKIntegrationMethod SKFilter::GetFilterIntegrationMethod(){
  return integrationMethod;
}

#ifdef FLOATDSP
void SKFilter::filter(float input){
  // noise term
  float noise;

  // feedback amount variables
  float res = 4.0f * Resonance;
  float fb = 0.0f;

  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6f * 2.0f * (noise - 0.5f);

  input += noise;

  // set filter mode
  switch(filterMode){
  case SK_LOWPASS_MODE:
    input_lp = input;
    input_bp = 0.0f;
    input_hp = 0.0f;
    break;
  case SK_BANDPASS_MODE:
    input_lp = 0.0f;
    input_bp = input;
    input_hp = 0.0f;
    break;
  case SK_HIGHPASS_MODE:
    input_lp = 0.0f;
    input_bp = 0.0f;
    input_hp = input;
    break;
  default:
    input_lp = 0.0f;
    input_bp = 0.0f;
    input_hp = 0.0f;
  }
    
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SK_SEMI_IMPLICIT_EULER:
      // semi-implicit euler integration
      {
	fb = input_bp + res * p1;
	p0 += dt * (input_lp - p0 - fb);
       	p1 += dt * (p0 + fb - p1 - 1.0f / 4.0f * FloatSinhPade54(p0 * 4.0f));
      	out = p1;
      }
      break;
    case SK_PREDICTOR_CORRECTOR:
      // predictor-corrector integration
      {
	float p0_prime, p1_prime, fb_prime;
	  
	fb = input_bp_t1 + res * p1;
	p0_prime = p0 + dt * (input_lp_t1 - p0 - fb);
       	p1_prime = p1 + dt * (p0 + fb - p1 - 1.0f / 4.0f * FloatSinhPade54(p1 * 4.0f));	
	fb_prime = input_bp + res * p1_prime;
	
       	p1 += 0.5 * dt * ((p0 + fb - p1 - 1.0f / 4.0f * FloatSinhPade54(p1 * 4.0f)) +
		      (p0_prime + fb_prime - p1_prime - 1.0f / 4.0f * FloatSinhPade54(p1 * 4.0f)));
	p0 += 0.5 * dt * ((input_lp_t1 - p0 - fb) +
		      (input_lp - p0_prime - fb_prime));

	out = p1;
      }
      break;
    case SK_TRAPEZOIDAL:
      // trapezoidal integration
      {
	float x_k, x_k2;
	float fb_t = input_bp_t1 + res*p1;
	float alpha = dt / 2.0f;
	float A = p0 + fb_t - p1 - 1.0f / 4.0f * FloatSinhPade54(4.0f * p1) +
	           p0 / (1.0f + alpha) + alpha / (1.0f + alpha) * (input_lp_t1 - p0 - fb_t + input_lp);
	float c = 1.0f - (alpha - alpha * alpha /(1.0f + alpha)) * res + alpha;
	float D_n = p1 + alpha * A + (alpha - alpha * alpha / (1.0f + alpha)) * input_bp;

	x_k = p1;
	
	// newton-raphson
	for(int ii=0; ii < SKF_MAX_NEWTON_STEPS; ii++) {
	  x_k2 = x_k - (c * x_k + alpha * 1.0f / 4.0f * FloatSinhPade54(4.0f * x_k) - D_n) / 
	                 (c + alpha * FloatCoshPade54(4.0f * x_k));
	  
#ifdef SKF_NEWTON_BREAKING_LIMIT
	  // breaking limit
	  if(fabs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
#endif	  
	  x_k = x_k2;
	}
	
	p1 = x_k;
	fb = input_bp + res * p1;
	p0 = p0 / (1.0f + alpha) + alpha / (1.0f + alpha) * (input_lp_t1 - p0 - fb_t + input_lp - fb);
	out = p1;
      }
      break;
    default:
      break;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter32(out);
    }
  }
  
  // set input at t-1
  input_lp_t1 = input_lp;    
  input_bp_t1 = input_bp;    
  input_hp_t1 = input_hp;    
}
#else
void SKFilter::filter(double input){
  // noise term
  double noise;

  // feedback amount variables
  double res=4.0*Resonance;
  double fb=0.0;

  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;

  // set filter mode
  switch(filterMode){
  case SK_LOWPASS_MODE:
    input_lp = input;
    input_bp = 0.0;
    input_hp = 0.0;
    break;
  case SK_BANDPASS_MODE:
    input_lp = 0.0;
    input_bp = input;
    input_hp = 0.0;
    break;
  case SK_HIGHPASS_MODE:
    input_lp = 0.0;
    input_bp = 0.0;
    input_hp = input;
    break;
  default:
    input_lp = 0.0;
    input_bp = 0.0;
    input_hp = 0.0;
  }
    
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SK_SEMI_IMPLICIT_EULER:
      // semi-implicit euler integration
      {
	fb = input_bp + res * p1;
	p0 += dt * (input_lp - p0 - fb);
       	p1 += dt * (p0 + fb - p1 - 1.0 / 4.0 * SinhPade34(p0 * 4.0));
      	out = p1;
      }
      break;
    case SK_PREDICTOR_CORRECTOR:
      // predictor-corrector integration
      {
	double p0_prime, p1_prime, fb_prime;
	  
	fb = input_bp_t1 + res * p1;
	p0_prime = p0 + dt * (input_lp_t1 - p0 - fb);
       	p1_prime = p1 + dt * (p0 + fb - p1 - 1.0 / 4.0 * SinhPade34(p1 * 4.0));	
	fb_prime = input_bp + res * p1_prime;
	
       	p1 += 0.5 * dt * ((p0 + fb - p1 - 1.0 / 4.0 * SinhPade34(p1 * 4.0)) +
		           (p0_prime + fb_prime - p1_prime - 1.0 / 4.0 * SinhPade34(p1 * 4.0)));
	p0 += 0.5 * dt * ((input_lp_t1 - p0 - fb) +
		           (input_lp - p0_prime - fb_prime));

	out = p1;
      }
      break;
    case SK_TRAPEZOIDAL:
      // trapezoidal integration
      {
	double x_k, x_k2;
	double fb_t = input_bp_t1 + res * p1;
	double alpha = dt / 2.0;
	double A = p0 + fb_t - p1 - 1.0 / 4.0 * SinhPade54(4.0 * p1) +
	           p0 / (1.0 + alpha) + alpha / (1 + alpha) * (input_lp_t1 - p0 - fb_t + input_lp);
	double c = 1.0 - (alpha - alpha * alpha / (1.0 + alpha)) * res + alpha;
	double D_n = p1 + alpha * A + (alpha - alpha * alpha / (1.0 + alpha)) * input_bp;

	x_k = p1;
	
	// newton-raphson
	for(int ii=0; ii < SKF_MAX_NEWTON_STEPS; ii++) {
	  x_k2 = x_k - (c * x_k + alpha * 1.0 / 4.0 * SinhPade54(4.0 * x_k) - D_n) /
	                 (c + alpha*CoshPade54(4.0 * x_k));
	  
#ifdef SKF_NEWTON_BREAKING_LIMIT
	  // breaking limit
	  if(fabs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
#endif	  
	  x_k = x_k2;
	}
	
	p1 = x_k;
	fb = input_bp + res * p1;
	p0 = p0 / (1.0 + alpha) + alpha / (1.0 + alpha) * (input_lp_t1 - p0 - fb_t + input_lp - fb);
	out = p1;
      }
      break;
    default:
      break;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
  
  // set input at t-1
  input_lp_t1 = input_lp;    
  input_bp_t1 = input_bp;    
  input_hp_t1 = input_hp;    
}
#endif

void SKFilter::SetFilterLowpassInput(double input){
  input_lp = input;
}

void SKFilter::SetFilterBandpassInput(double input){
  input_bp = input;
}

void SKFilter::SetFilterHighpassInput(double input){
  input_hp = input;
}
