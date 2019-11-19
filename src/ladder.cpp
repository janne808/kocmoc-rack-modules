/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Ladder Filter VCV Rack plugin.
 *
 *  Ladder Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ladder Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ladder Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <cmath>
#include "ladder.h"
#include "fir.h"

// constructor
Ladder::Ladder(double newCutoff, double newResonance, int newOversamplingFactor,
	       LadderFilterMode newFilterMode, double newSampleRate, LadderIntegrationMethod newIntegrationMethod){
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
  p2 = 0.0;
  p3 = 0.0;
  out = 0.0;
  ut_1 = 0.0;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default constructor
Ladder::Ladder(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = LADDER_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  p2 = 0.0;
  p3 = 0.0;
  out = 0.0;
  ut_1 = 0.0;
  
  integrationMethod = LADDER_PREDICTOR_CORRECTOR_FULL_TANH;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default destructor
Ladder::~Ladder(){
  delete fir;
}

void Ladder::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  p2 = 0.0;
  p3 = 0.0;
  out = 0.0;
  ut_1 = 0.0;
  
  // set oversampling
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));
}

void Ladder::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void Ladder::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void Ladder::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void Ladder::SetFilterMode(LadderFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void Ladder::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  fir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void Ladder::SetFilterIntegrationMethod(LadderIntegrationMethod method){
  integrationMethod = method;
}

void Ladder::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 0.6){
    dt = 0.6;
  }
}

// pade 3/2 approximant for tanh
inline double Ladder::Tanh32(double x) {
  // clamp x to -3..3
  if(x > 3.0) {
    x=3.0;
  }
  else if(x < -3.0) {
    x=-3.0;
  }

  // return approximant
  return x*(15.0+x*x)/(15.0+6.0*x*x);
}

// pade 5/4 approximant for tanh
inline double Ladder::Tanh54(double x) {
  // clamp x to -4..4
  if(x > 4.0) {
    x=4.0;
  }
  else if(x < -4.0) {
    x=-4.0;
  }
  
  // return approximant
  return x*(945.0+105.0*x*x+x*x*x*x)/(945.0+420.0*x*x+15.0*x*x*x*x);
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

double Ladder::GetFilterOutput(){
  return out;
}

LadderFilterMode Ladder::GetFilterMode(){
  return filterMode;
}

double Ladder::GetFilterSampleRate(){
  return sampleRate;
}

LadderIntegrationMethod Ladder::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void Ladder::LadderFilter(double input){
  // noise term
  double noise;

  // feedback amount
  double fb = 8.0*Resonance;

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
      {
	// semi-implicit euler integration
	// with full tanh stages
	p0 = p0 + dt*(Tanh32(input - fb*p3) - p0);
	p0 = Tanh32(p0);
	p1 = p1 + dt*(p0 - p1);
	p1 = Tanh32(p1);
	p2 = p2 + dt*(p1 - p2);
	p2 = Tanh32(p2);
	p3 = p3 + dt*(p2 - p3);
	p3 = Tanh32(p3);
      }
      break;
    case LADDER_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(Tanh32(ut_1 - fb*p3) - p0);
	p0_prime = Tanh32(p0_prime);
	p1_prime = p1 + dt*(p0 - p1);
	p1_prime = Tanh32(p1_prime);
	p2_prime = p2 + dt*(p1 - p2);
	p2_prime = Tanh32(p2_prime);
	p3_prime = p3 + dt*(p2 - p3);
	p3_prime = Tanh32(p3_prime);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((p2 - p3) + (p2_prime - p3_prime));
	p3 = Tanh32(p3);
	p2 = p2 + 0.5*dt*((p1 - p2) + (p1_prime - p2_prime));
	p2 = Tanh32(p2);
	p1 = p1 + 0.5*dt*((p0 - p1) + (p0_prime - p1_prime));
	p1 = Tanh32(p1);
	p0 = p0 + 0.5*dt*((Tanh32(ut_1 - fb*p3t_1) - p0) +
			  (Tanh32(input - fb*p3) - p0_prime));
	p0 = Tanh32(p0);
      }
      break;
    case LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH:
      // predictor-corrector integration
      // with feedback tanh stage only
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(Tanh32(ut_1 - fb*p3) - p0);
	p1_prime = p1 + dt*(p0 - p1);
	p2_prime = p2 + dt*(p1 - p2);
	p3_prime = p3 + dt*(p2 - p3);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((p2 - p3) + (p2_prime - p3_prime));
	p2 = p2 + 0.5*dt*((p1 - p2) + (p1_prime - p2_prime));
	p1 = p1 + 0.5*dt*((p0 - p1) + (p0_prime - p1_prime));
	p0 = p0 + 0.5*dt*((Tanh32(ut_1 - fb*p3t_1) - p0) +
			  (Tanh32(input - fb*p3) - p0_prime));
      }
      break;
    case LADDER_TRAPEZOIDAL_FEEDBACK_TANH:
      // implicit trapezoidal integration
      // with feedback tanh stage only
      {
	double x_k, x_k2, g, b, c, C_t, D_t, ut, ut_2;
	double p0_prime, p1_prime, p2_prime, p3_prime;

	ut = Tanh32(ut_1 - fb*p3);
    	b = (0.5*dt)/(1.0 + 0.5*dt);
	c = (1.0-0.5*dt)/(1.0+0.5*dt);
	g = -fb*b*b*b*b;
	x_k = ut;
	D_t = c*p3 + (b + c*b)*p2 + (b*b+b*b*c)*p1 + (b*b*b+b*b*b*c)*p0 + b*b*b*b*ut;
	C_t = Tanh32(input - fb*D_t);

	// newton-raphson 
	for(int ii=0; ii < 32; ii++) {
	  double tanh_g_xk, tanh_g_xk2;
	  
	  tanh_g_xk = Tanh32(g*x_k);
	  tanh_g_xk2 = g*(1.0 - Tanh32(g*x_k)*Tanh32(g*x_k));
	  
	  x_k2 = x_k - (x_k + x_k*tanh_g_xk*C_t - tanh_g_xk - C_t) /
	                 (1.0 + C_t*(tanh_g_xk + x_k*tanh_g_xk2) - tanh_g_xk2);
	  
	  // breaking limit
	  if(abs(x_k2 - x_k) < 1.0e-15) {
	    x_k = x_k2;
	    break;
	  }
	  
	  x_k = x_k2;
	}
	
	ut_2 = x_k;

	p0_prime = p0;
	p1_prime = p1;
	p2_prime = p2;
	p3_prime = p3;

	p0 = c*p0_prime + b*(ut + ut_2);
	p1 = c*p1_prime + b*(p0_prime + p0);
	p2 = c*p2_prime + b*(p1_prime + p1);
	p3 = c*p3_prime + b*(p2_prime + p2);
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
      out = Tanh32(input - p0 - fb*p3);
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = fir->FIRfilter(out);
    }
  }
}

double Ladder::GetFilterLowpass(){
  return p3;
}

double Ladder::GetFilterBandpass(){
  return 0.0;
}

double Ladder::GetFilterHighpass(){
  return 0.0;
}


