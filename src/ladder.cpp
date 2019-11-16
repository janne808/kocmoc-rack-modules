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
	       int newFilterMode, double newSampleRate, int newIntegrationMethod){
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
  filterMode = 0;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = 0.0;
  p1 = 0.0;
  p2 = 0.0;
  p3 = 0.0;
  out = 0.0;
  ut_1 = 0.0;
  
  integrationMethod = 0;
  
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

void Ladder::SetFilterMode(int newFilterMode){
  filterMode = newFilterMode;
}

void Ladder::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  fir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void Ladder::SetFilterIntegrationMethod(int method){
  integrationMethod = method;
}

void Ladder::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clip integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  if(dt > 0.8){
    dt = 0.8;
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

double Ladder::GetFilterOutput(){
  return out;
}

int Ladder::GetFilterMode(){
  return filterMode;
}

double Ladder::GetFilterSampleRate(){
  return sampleRate;
}

int Ladder::GetFilterIntegrationMethod(){
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

  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case 0:
      {
	// semi-implicit euler integration
	// with full tanh stages
	p0 = p0 + dt*((input - std::tanh(fb*p3) + noise) - p0);
	p0 = std::tanh(p0);
	p1 = p1 + dt*(p0 - p1);
	p1 = std::tanh(p1);
	p2 = p2 + dt*(p1 - p2);
	p2 = std::tanh(p2);
	p3 = p3 + dt*(p2 - p3);
	p3 = std::tanh(p3);
      }
      break;
    case 1:
      // predictor-corrector integration
      // with full tanh stages
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(ut_1 - std::tanh(fb*p3) - p0);
	p0_prime = std::tanh(p0_prime);
	p1_prime = p1 + dt*(p0 - p1);
	p1_prime = std::tanh(p1_prime);
	p2_prime = p2 + dt*(p1 - p2);
	p2_prime = std::tanh(p2_prime);
	p3_prime = p3 + dt*(p2 - p3);
	p3_prime = std::tanh(p3_prime);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((p2 - p3) + (p2_prime - p3_prime));
	p3 = std::tanh(p3);
	p2 = p2 + 0.5*dt*((p1 - p2) + (p1_prime - p2_prime));
	p2 = std::tanh(p2);
	p1 = p1 + 0.5*dt*((p0 - p1) + (p0_prime - p1_prime));
	p1 = std::tanh(p1);
	p0 = p0 + 0.5*dt*((ut_1 - std::tanh(fb*p3t_1) - p0) + (input - std::tanh(fb*p3) - p0_prime));
	p0 = std::tanh(p0);
      }
      break;
    case 2:
      // predictor-corrector integration
      // with feedback tanh stage only
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt*(ut_1 - std::tanh(fb*p3) - p0);
	p1_prime = p1 + dt*(p0 - p1);
	p2_prime = p2 + dt*(p1 - p2);
	p3_prime = p3 + dt*(p2 - p3);

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5*dt*((p2 - p3) + (p2_prime - p3_prime));
	p2 = p2 + 0.5*dt*((p1 - p2) + (p1_prime - p2_prime));
	p1 = p1 + 0.5*dt*((p0 - p1) + (p0_prime - p1_prime));
	p0 = p0 + 0.5*dt*((ut_1 - std::tanh(fb*p3t_1) - p0) + (input - std::tanh(fb*p3) - p0_prime));
      }
      break;
    default:
      break;
    }

    // input at t-1
    ut_1 = input;
    
    switch(filterMode){
    case 0:
      out = p3;
      break;
    case 1:
      out = p1 - p3;
      break;
    case 2:
      out = input - p0 - fb*p3;
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


