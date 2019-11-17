/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of State Variable Filter VCV Rack plugin.
 *
 *  State Variable Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  State Variable Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with State Variable Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <cmath>
#include "svf.h"
#include "fir.h"

// constructor
SVF::SVF(double newCutoff, double newResonance, int newOversamplingFactor,
	 SVFFilterMode newFilterMode, double newSampleRate, SVFIntegrationMethod newIntegrationMethod){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  oversamplingFactor = newOversamplingFactor;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;

  SetFilterIntegrationRate();

  // initialize filter state
  hp = 0.0;
  bp = 0.0;
  lp = 0.0;
  out = 0.0;
  u_t1 = 0.0;

  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default constructor
SVF::SVF(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = SVF_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  hp = 0.0;
  bp = 0.0;
  lp = 0.0;
  out = 0.0;
  u_t1 = 0.0;

  integrationMethod = SVF_SEMI_IMPLICIT_EULER;
  
  // instantiate downsampling filter
  fir = new FIRLowpass(sampleRate * oversamplingFactor, (sampleRate / (double)(oversamplingFactor)), 32);
}

// default destructor
SVF::~SVF(){
  delete fir;
}

void SVF::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;

  SetFilterIntegrationRate();
  
  // initialize filter state
  hp = 0.0;
  bp = 0.0;
  lp = 0.0;
  out = 0.0;
  u_t1 = 0.0;

  // set oversampling
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));
}

void SVF::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void SVF::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void SVF::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  fir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void SVF::SetFilterMode(SVFFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void SVF::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  fir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  fir->SetFilterCutoff((sampleRate / (double)(oversamplingFactor)));

  SetFilterIntegrationRate();
}

void SVF::SetFilterIntegrationMethod(SVFIntegrationMethod method){
  integrationMethod = method;
}

void SVF::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > 1.2){
    dt = 1.2;
  }
}

// pade 3/2 approximant for tanh
inline double SVF::Tanh32(double x) {
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
inline double SVF::Tanh54(double x) {
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

double SVF::GetFilterCutoff(){
  return cutoffFrequency;
}

double SVF::GetFilterResonance(){
  return Resonance;
}

int SVF::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

double SVF::GetFilterOutput(){
  return out;
}

SVFFilterMode SVF::GetFilterMode(){
  return filterMode;
}

double SVF::GetFilterSampleRate(){
  return sampleRate;
}

SVFIntegrationMethod SVF::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void SVF::SVFfilter(double input){
  // noise term
  double noise;

  // feedback amount
  double fb = 1.0 - 2.0 * Resonance;
  
  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SVF_SEMI_IMPLICIT_EULER:
      // semi-implicit euler integration
      hp = -lp - fb*bp + input + noise;
      bp += dt*hp;
      bp = Tanh32(bp);
      lp += dt*bp;
      lp = Tanh32(lp);
      break;
    case SVF_PREDICTOR_CORRECTOR:
      // predictor-corrector integration
      double hp_prime, bp_prime, hp2;

      // predictor
      hp_prime =  -lp - fb*bp + u_t1 + noise;
      bp_prime = bp + dt*hp_prime;
      bp_prime = Tanh32(bp_prime);

      // corrector
      lp += 0.5 * dt * (bp + bp_prime);
      lp = Tanh32(lp);
      hp2 = -lp - fb*bp_prime + input + noise;
      bp += 0.5 * dt * (hp_prime + hp2);
      bp = Tanh32(bp);
      hp = -lp - fb*bp + input;
      break;
    case SVF_TRAPEZOIDAL:
      // trapezoidal integration
      double a, b, c, bp0;
      
      a = (1.0 - 0.5*fb*dt - 0.25*dt*dt) / (1.0 + 0.5*fb*dt + 0.25*dt*dt);
      b = dt / (1 + 0.5*fb*dt + 0.25*dt*dt);
      c = dt / (2.0 + fb*dt + 0.5*dt*dt);
      bp0 = bp;
      bp = a*bp - b*lp + c*(input + u_t1);
      bp = Tanh32(bp);
      lp += 0.5*dt*(bp0 + bp);
      lp = Tanh32(lp);
      hp = -lp - fb*bp + input;
      break;
    default:
      break;
    }
    
    // set input at t-1
    u_t1 = input;
    
    switch(filterMode){
    case SVF_LOWPASS_MODE:
      out = lp;
      break;
    case SVF_BANDPASS_MODE:
      out = bp;
      break;
    case SVF_HIGHPASS_MODE:
      out = hp;
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = fir->FIRfilter(out) * 0.4;
    }
  }
}

double SVF::GetFilterLowpass(){
  return lp;
}

double SVF::GetFilterBandpass(){
  return bp;
}

double SVF::GetFilterHighpass(){
  return hp;
}

