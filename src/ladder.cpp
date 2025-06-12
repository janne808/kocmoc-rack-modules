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
#include "ladder.h"

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 16

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.75

// maximum newton-raphson iteration steps
#define LADDER_MAX_NEWTON_STEPS 8

// check for newton-raphson breaking limit
#define LADDER_NEWTON_BREAKING_LIMIT 1

// constructor
Ladder::Ladder(double newCutoff, double newResonance, int newOversamplingFactor,
	       LadderFilterMode newFilterMode, double newSampleRate,
	       LadderIntegrationMethod newIntegrationMethod, int newDecimatorOrder){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;
  oversamplingFactor = newOversamplingFactor;
  decimatorOrder = newDecimatorOrder;
  
  SetFilterIntegrationRate();

  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#endif
}

// default constructor
Ladder::Ladder(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  filterMode = LADDER_LOWPASS_MODE;
  sampleRate = 44100.0;
  oversamplingFactor = 2;
  decimatorOrder = IIR_DOWNSAMPLE_ORDER;
  
  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  integrationMethod = LADDER_PREDICTOR_CORRECTOR_FULL_TANH;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#endif
}

// default destructor
Ladder::~Ladder(){
  delete iir;
}

void Ladder::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);  
}

void Ladder::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void Ladder::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void Ladder::SetFilterMode(LadderFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void Ladder::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);

  SetFilterIntegrationRate();
}

void Ladder::SetFilterIntegrationMethod(LadderIntegrationMethod method){
  integrationMethod = method;
}

void Ladder::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);

  SetFilterIntegrationRate();
}

void Ladder::SetFilterDecimatorOrder(int newDecimatorOrder){
  decimatorOrder = newDecimatorOrder;
  iir->SetFilterOrder(decimatorOrder);
}

void Ladder::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0f / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0f){
    dt = 0.0f;
  }
  else if(dt > 0.7f){
    dt = 0.7f;
  }
}

double Ladder::GetFilterCutoff(){
  return cutoffFrequency;
}

double Ladder::GetFilterResonance(){
  return Resonance;
}

int Ladder::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

int Ladder::GetFilterDecimatorOrder(){
  return decimatorOrder;
}

#ifdef FLOATDSP
float Ladder::GetFilterOutput(){
  return out;
}
#else
double Ladder::GetFilterOutput(){
  return out;
}
#endif

LadderFilterMode Ladder::GetFilterMode(){
  return filterMode;
}

double Ladder::GetFilterSampleRate(){
  return sampleRate;
}

LadderIntegrationMethod Ladder::GetFilterIntegrationMethod(){
  return integrationMethod;
}

#ifdef FLOATDSP
void Ladder::LadderFilter(float input){
  // noise term
  float noise;

  // feedback amount
  float fb = 6.0f * Resonance;

  // integration rate
  float dt2 = dt;
  
  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6f * 2.0f * (noise - 0.5f);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case LADDER_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + dt2 * (FloatTanhPade45(input - fb * p3) - FloatTanhPade45(p0));
	p1 = p1 + dt2 * (FloatTanhPade45(p0) - FloatTanhPade45(p1));
	p2 = p2 + dt2 * (FloatTanhPade45(p1) - FloatTanhPade45(p2));
	p3 = p3 + dt2 * (FloatTanhPade45(p2) - FloatTanhPade45(p3));
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	// euler step nonlinearities
	float tanh_ut1_fb_p3 = FloatTanhPade45(ut_1 - fb * p3);
	float tanh_p0 = FloatTanhPade45(p0);
	float tanh_p1 = FloatTanhPade45(p1);
	float tanh_p2 = FloatTanhPade45(p2);
	float tanh_p3 = FloatTanhPade45(p3);

	// euler step differences
	float p0_euler = tanh_ut1_fb_p3 - tanh_p0;
	float p1_euler = tanh_p0 - tanh_p1;
	float p2_euler = tanh_p1 - tanh_p2;
	float p3_euler = tanh_p2 - tanh_p3;
	
	// predictor
	float p0_prime = p0 + dt * p0_euler;
	float p1_prime = p1 + dt * p1_euler;
	float p2_prime = p2 + dt * p2_euler;
	float p3_prime = p3 + dt * p3_euler;

	// trapezoidal step nonlinearities
	float tanh_input_fb_p3_prime = FloatTanhPade45(input - fb * p3_prime);
	float tanh_p0_prime = FloatTanhPade45(p0_prime);
	float tanh_p1_prime = FloatTanhPade45(p1_prime);
	float tanh_p2_prime = FloatTanhPade45(p2_prime);
	float tanh_p3_prime = FloatTanhPade45(p3_prime);

	// trapezoidal step differences
	float p0_trap = tanh_input_fb_p3_prime - tanh_p0_prime;
	float p1_trap = tanh_p0_prime - tanh_p1_prime;
	float p2_trap = tanh_p1_prime - tanh_p2_prime;
	float p3_trap = tanh_p2_prime - tanh_p3_prime;
	
	// corrector
	p0 = p0 + 0.5 * dt * (p0_euler + p0_trap);
	p1 = p1 + 0.5 * dt * (p1_euler + p1_trap);
	p2 = p2 + 0.5 * dt * (p2_euler + p2_trap);
	p3 = p3 + 0.5 * dt * (p3_euler + p3_trap);
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH:
      // predictor-corrector integration
      // with feedback tanh stage only
      {
	float p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt2 * (FloatTanhPade45(ut_1 - fb * p3) - p0);
	p1_prime = p1 + dt2 * (p0 - p1);
	p2_prime = p2 + dt2 * (p1 - p2);
	p3_prime = p3 + dt2 * (p2 - p3);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5f * dt2 * ((p2 - p3) + (p2_prime - p3_prime));
	p2 = p2 + 0.5f * dt2 * ((p1 - p2) + (p1_prime - p2_prime));
	p1 = p1 + 0.5f * dt2 * ((p0 - p1) + (p0_prime - p1_prime));
	p0 = p0 + 0.5f * dt2 * ((FloatTanhPade45(ut_1 - fb * p3t_1) - p0) +
			          (FloatTanhPade45(input - fb * p3) - p0_prime));
      }
      break;
      
    case LADDER_TRAPEZOIDAL_FEEDBACK_TANH:
      // implicit trapezoidal integration
      // with feedback tanh stage only
      {
	float x_k, x_k2, g, b, c, C_t, D_t, ut, ut_2;
	float p0_prime, p1_prime, p2_prime, p3_prime;

	ut = FloatTanhPade45(ut_1 - fb * p3);
    	b = (0.5f * dt2) / (1.0f + 0.5f * dt2);
	c = (1.0f - 0.5f * dt2) / (1.0f + 0.5f * dt2);
	g = -1.0f * fb * b * b * b * b;
	x_k = ut;
	D_t = c*p3 + (b + c * b) * p2 + (b * b + b * b * c) * p1 +
	               (b * b * b+b * b * b * c)*p0 + b * b * b * b * ut;
	C_t = FloatTanhPade45(input - fb * D_t);

	// newton-raphson 
	for(int ii=0; ii < LADDER_MAX_NEWTON_STEPS; ii++) {
	  float tanh_g_xk, tanh_g_xk2;
	  
	  tanh_g_xk = FloatTanhPade45(g * x_k);
	  tanh_g_xk2 = g * (1.0f - FloatTanhPade45(g * x_k) * FloatTanhPade45(g * x_k));
	  
	  x_k2 = x_k - (x_k + x_k * tanh_g_xk * C_t - tanh_g_xk - C_t) /
	                 (1.0f + C_t * (tanh_g_xk + x_k * tanh_g_xk2) - tanh_g_xk2);
	  
#ifdef LADDER_NEWTON_BREAKING_LIMIT
	  // breaking limit
	  if(fabs(x_k2 - x_k) < 1.0e-9f) {
	    x_k = x_k2;
	    break;
	  }
#endif	  
	  x_k = x_k2;
	}
	
	ut_2 = x_k;

	p0_prime = p0;
	p1_prime = p1;
	p2_prime = p2;
	p3_prime = p3;

	p0 = c * p0_prime + b * (ut + ut_2);
	p1 = c * p1_prime + b * (p0_prime + p0);
	p2 = c * p2_prime + b * (p1_prime + p1);
	p3 = c * p3_prime + b * (p2_prime + p2);
      }
      break;
      
    default:
      break;
    }

    // input at t-1
    ut_1 = input;

    //switch filter mode
    switch(filterMode){
    case LADDER_LOWPASS_MODE:
      out = p3;
      break;
    case LADDER_BANDPASS_MODE:
      out = p1 - p3;
      break;
    case LADDER_HIGHPASS_MODE:
      out = FloatTanhPade45(input - p0 - fb * p3);
      break;
    default:
      out = 0.0f;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter32(out);
    }
  }
}
#else
void Ladder::LadderFilter(double input){
  // noise term
  double noise;

  // feedback amount
  double fb = 6.0 * Resonance;

  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case LADDER_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + dt * (TanhPade32(input - fb * p3) - TanhPade32(p0));
	p1 = p1 + dt * (TanhPade32(p0) - TanhPade32(p1));
	p2 = p2 + dt * (TanhPade32(p1) - TanhPade32(p2));
	p3 = p3 + dt * (TanhPade32(p2) - TanhPade32(p3));
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	// euler step nonlinearities
	double tanh_ut1_fb_p3 = TanhPade32(ut_1 - fb * p3);
	double tanh_p0 = TanhPade32(p0);
	double tanh_p1 = TanhPade32(p1);
	double tanh_p2 = TanhPade32(p2);
	double tanh_p3 = TanhPade32(p3);

	// euler step differences
	double p0_euler = tanh_ut1_fb_p3 - tanh_p0;
	double p1_euler = tanh_p0 - tanh_p1;
	double p2_euler = tanh_p1 - tanh_p2;
	double p3_euler = tanh_p2 - tanh_p3;
	
	// predictor
	double p0_prime = p0 + dt * p0_euler;
	double p1_prime = p1 + dt * p1_euler;
	double p2_prime = p2 + dt * p2_euler;
	double p3_prime = p3 + dt * p3_euler;

	// trapezoidal step nonlinearities
	double tanh_input_fb_p3_prime = TanhPade32(input - fb * p3_prime);
	double tanh_p0_prime = TanhPade32(p0_prime);
	double tanh_p1_prime = TanhPade32(p1_prime);
	double tanh_p2_prime = TanhPade32(p2_prime);
	double tanh_p3_prime = TanhPade32(p3_prime);

	// trapezoidal step differences
	double p0_trap = tanh_input_fb_p3_prime - tanh_p0_prime;
	double p1_trap = tanh_p0_prime - tanh_p1_prime;
	double p2_trap = tanh_p1_prime - tanh_p2_prime;
	double p3_trap = tanh_p2_prime - tanh_p3_prime;
	
	// corrector
	p0 = p0 + 0.5 * dt * (p0_euler + p0_trap);
	p1 = p1 + 0.5 * dt * (p1_euler + p1_trap);
	p2 = p2 + 0.5 * dt * (p2_euler + p2_trap);
	p3 = p3 + 0.5 * dt * (p3_euler + p3_trap);
      }
      break;
      
    case LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH:
      // predictor-corrector integration
      // with feedback tanh stage only
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt * (TanhPade32(ut_1 - fb * p3) - p0);
	p1_prime = p1 + dt * (p0 - p1);
	p2_prime = p2 + dt * (p1 - p2);
	p3_prime = p3 + dt * (p2 - p3);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5 * dt * ((p2 - p3) + (p2_prime - p3_prime));
	p2 = p2 + 0.5 * dt * ((p1 - p2) + (p1_prime - p2_prime));
	p1 = p1 + 0.5 * dt * ((p0 - p1) + (p0_prime - p1_prime));
	p0 = p0 + 0.5 * dt * ((TanhPade32(ut_1 - fb * p3t_1) - p0) +
			       (TanhPade32(input - fb * p3_prime) - p0_prime));
      }
      break;
      
    case LADDER_TRAPEZOIDAL_FEEDBACK_TANH:
      // implicit trapezoidal integration
      // with feedback tanh stage only
      {
	double x_k, x_k2, g, b, c, C_t, D_t, ut, ut_2;
	double p0_prime, p1_prime, p2_prime, p3_prime;

	ut = TanhPade32(ut_1 - fb * p3);
    	b = (0.5 * dt) / (1.0 + 0.5 * dt);
	c = (1.0 - 0.5 * dt) / (1.0 + 0.5 * dt);
	g = -fb * b * b * b * b;
	x_k = ut;
	D_t = c * p3 + (b + c * b) * p2 + (b * b + b * b * c) * p1 +
	               (b * b * b + b * b * b * c) * p0 + b * b * b * b * ut;
	C_t = TanhPade32(input - fb * D_t);

	// newton-raphson 
	for(int ii=0; ii < LADDER_MAX_NEWTON_STEPS; ii++) {
	  double tanh_g_xk, tanh_g_xk2;
	  
	  tanh_g_xk = TanhPade32(g * x_k);
	  tanh_g_xk2 = g * (1.0 - TanhPade32(g * x_k) * TanhPade32(g * x_k));
	  
	  x_k2 = x_k - (x_k + x_k * tanh_g_xk * C_t - tanh_g_xk - C_t) /
	                 (1.0 + C_t * (tanh_g_xk + x_k * tanh_g_xk2) - tanh_g_xk2);
	  
#ifdef LADDER_NEWTON_BREAKING_LIMIT
	  // breaking limit
	  if(fabs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
#endif	  
	  x_k = x_k2;
	}
	
	ut_2 = x_k;

	p0_prime = p0;
	p1_prime = p1;
	p2_prime = p2;
	p3_prime = p3;

	p0 = c * p0_prime + b * (ut + ut_2);
	p1 = c * p1_prime + b * (p0_prime + p0);
	p2 = c * p2_prime + b * (p1_prime + p1);
	p3 = c * p3_prime + b * (p2_prime + p2);
      }
      break;
      
    default:
      break;
    }

    // input at t-1
    ut_1 = input;

    //switch filter mode
    switch(filterMode){
    case LADDER_LOWPASS_MODE:
      out = p3;
      break;
    case LADDER_BANDPASS_MODE:
      out = p1 - p3;
      break;
    case LADDER_HIGHPASS_MODE:
      out = TanhPade32(input - p0 - fb * p3);
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
}
#endif

double Ladder::GetFilterLowpass(){
  return p3;
}

double Ladder::GetFilterBandpass(){
  return 0.0;
}

double Ladder::GetFilterHighpass(){
  return 0.0;
}


