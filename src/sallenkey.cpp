/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Sallen-Key Filter VCV Rack plugin.
 *
 *  Sallen-Key Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Sallen-Key Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Sallen-Key Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <cmath>
#include "sallenkey.h"
#include "fir.h"

// constructor
SKFilter::SKFilter(double newCutoff, double newResonance, int newOversamplingFactor,
		   SKFilterMode newFilterMode, double newSampleRate, SKIntegrationMethod newIntegrationMethod){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  oversamplingFactor = newOversamplingFactor;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;

  SetFilterIntegrationRate();

  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  out = 0.0;

  // initialize filter inputs
  input_lp = 0.0;
  input_bp = 0.0;
  input_hp = 0.0;
  input_lp_t1 = 0.0;
  input_bp_t1 = 0.0;
  input_hp_t1 = 0.0;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default constructor
SKFilter::SKFilter(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = SK_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  out = 0.0;
  
  // initialize filter inputs
  input_lp = 0.0;
  input_bp = 0.0;
  input_hp = 0.0;
  input_lp_t1 = 0.0;
  input_bp_t1 = 0.0;
  input_hp_t1 = 0.0;
  
  integrationMethod = SK_SEMI_IMPLICIT_EULER;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default destructor
SKFilter::~SKFilter(){
  delete fir;
}

void SKFilter::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  out = 0.0;

  // initialize filter inputs
  input_lp = 0.0;
  input_bp = 0.0;
  input_hp = 0.0;
  input_lp_t1 = 0.0;
  input_bp_t1 = 0.0;
  input_hp_t1 = 0.0;
  
  
  // set oversampling
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));
}

void SKFilter::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void SKFilter::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterMode(SKFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void SKFilter::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  fir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void SKFilter::SetFilterIntegrationMethod(SKIntegrationMethod method){
  integrationMethod = method;
}

void SKFilter::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 0.3){
    dt = 0.3;
  }
}

// pade 3/2 approximant for tanh
inline double SKFilter::TanhPade32(double x) {
  // clamp x to -3..3
  if(x > 3.0) {
    x = 3.0;
  }
  else if(x < -3.0) {
    x = -3.0;
  }
  // return approximant
  return x*(15.0 + x*x)/(15.0 + 6.0*x*x);
}

// pade 5/4 approximant for tanh
inline double SKFilter::TanhPade54(double x) {
  // clamp x to -4..4
  if(x > 4.0) {
    x = 4.0;
  }
  else if(x < -4.0) {
    x = -4.0;
  }
  // return approximant
  return x*(945.0 + 105.0*x*x+x*x*x*x)/(945.0 + 420.0*x*x + 15.0*x*x*x*x);
}

inline double SKFilter::ExpTaylor(double x, int N) {
  double y = 1.0;
  double e = x;
  double f = 1.0;
  
  // iterate taylor expansion up to N
  for(int ii=0; ii<N; ii++) {
    f *= ii+1;
    y += e/f;
    e *= x;
  }
  
  return y;
}

inline double SKFilter::BramSaturator(double x, double a) {
  double absX;
  double out;

  if(x < 0.0) {
    absX = -x;
  }
  else {
    absX = x;
  }
  
  if(absX < a) {
    out = absX;
  }
  else if(absX > 1.0) {
    out = (a+1.0)/2.0;
  }
  else {
    out = (absX - a)/(1.0 - a);
    out = a + (absX - a)/(1.0 + out*out);
  }

  if(x < 0.0) {
    out *= -1.0;
  }

  return out;
}

inline double SKFilter::TanhExpTaylor(double x, int N) {
  double e;

  // clamp x to -3..3
  if(x > 3.0) {
    x = 3.0;
  }
  else if(x < -3.0) {
    x = -3.0;
  }
  
  e = ExpTaylor(2.0*x, N);
  
  return (e - 1.0)/(e + 1.0);
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

double SKFilter::GetFilterOutput(){
  return out;
}

SKFilterMode SKFilter::GetFilterMode(){
  return filterMode;
}

double SKFilter::GetFilterSampleRate(){
  return sampleRate;
}

SKIntegrationMethod SKFilter::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void SKFilter::filter(double input){
  // noise term
  double noise;

  // feedback amount variables
  double res=3.5*Resonance;
  double fb=0.0;

  // antisaturator lambda function
  auto AntiSaturator = [](double input, double drive){ return 1.0/drive*sinh(input*drive); };

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
	fb = input_bp + res*p1;
	p0 += dt*(input_lp - p0 - fb);
       	p1 += dt*(p0 + fb - p1 - AntiSaturator(p1, 4.0));
      	out = p1;
      }
      break;
    case SK_PREDICTOR_CORRECTOR:
      // predictor-corrector integration
      {
	double p0_prime, p1_prime, fb_prime;
	  
	fb = input_bp_t1 + res*p1;
	p0_prime = p0 + dt*(input_lp_t1 - p0 - fb);
       	p1_prime = p1 + dt*(p0 + fb - p1 - AntiSaturator(p1, 4.0));	
	fb_prime = input_bp + res*p1_prime;
	
       	p1 += 0.5*dt*((p0 + fb - p1 - AntiSaturator(p1, 4.0)) +
		      (p0_prime + fb_prime - p1_prime - AntiSaturator(p1, 4.0)));
	p0 += 0.5*dt*((input_lp_t1 - p0 - fb) +
		      (input_lp - p0_prime - fb_prime));

	out = p1;
      }
      break;
    case SK_TRAPEZOIDAL:
      // trapezoidal integration
      {
	double x_k, x_k2;
	double fb_t = input_bp_t1 + res*p1;
	double alpha = dt/2.0;
	double A = p0 + fb_t - p1 - 1.0/4.0*sinh(4.0*p1) +
	           p0/(1.0 + alpha) + alpha/(1 + alpha)*(input_lp_t1 - p0 - fb_t + input_lp);
	double c = 1.0 - (alpha + alpha*alpha/(1.0 + alpha))*res + alpha;
	double D_n = p1 + alpha*A + (alpha + alpha*alpha/(1.0 + alpha))*input_bp;

	x_k = p1;
	
	// newton-raphson
	for(int ii=0; ii < 32; ii++) {
	  x_k2 = x_k - (c*x_k + alpha*1.0/4.0*sinh(4.0*x_k) - D_n)/(c + alpha*cosh(4.0*x_k));
	  
	  // breaking limit
	  if(abs(x_k2 - x_k) < 1.0e-15) {
	    x_k = x_k2;
	    break;
	  }
	  
	  x_k = x_k2;
	}
	
	p1 = x_k;
	fb = input_bp + res*p1;
	p0 = p0/(1.0 + alpha) + alpha/(1.0 + alpha)*(input_lp_t1 - p0 - fb_t + input_lp - fb);
	out = p1;
      }
      break;
    default:
      break;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = fir->FIRfilter(out) * 0.4;
    }
  }
  
  // set input at t-1
  input_lp_t1 = input_lp;    
  input_bp_t1 = input_bp;    
  input_hp_t1 = input_hp;    
}

void SKFilter::SetFilterLowpassInput(double input){
  input_lp = input;
}

void SKFilter::SetFilterBandpassInput(double input){
  input_bp = input;
}

void SKFilter::SetFilterHighpassInput(double input){
  input_hp = input;
}
