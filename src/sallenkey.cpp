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
SKFilter::SKFilter(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = SK_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  hp = 0.0;
  bp = 0.0;
  lp = 0.0;
  out = 0.0;
  u_t1 = 0.0;
  
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
  hp = 0.0;
  bp = 0.0;
  lp = 0.0;
  out = 0.0;
  u_t1 = 0.0;

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
  else if(dt > 1.2){
    dt = 1.2;
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
  double fb;

  // feedback amount
  fb = 2.5*Resonance;

  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SK_SEMI_IMPLICIT_EULER:
      // semi-implicit euler integration
      {
	hp = fb*lp;
	bp += dt*(input-hp - bp);
	bp *= 1.0 - (0.0025/oversamplingFactor);	
	lp += dt*(BramSaturator(bp + hp, 0.5) - BramSaturator(lp, 0.5));
      }
      break;
    default:
      break;
    }
    
    switch(filterMode){
    case SK_LOWPASS_MODE:
      out = lp;
      break;
    case SK_BANDPASS_MODE:
      out = bp;
      break;
    case SK_HIGHPASS_MODE:
      out = input - lp;
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = fir->FIRfilter(out) * 0.4;
    }
  }
  
  // set input at t-1
  u_t1 = input;    
}

double SKFilter::GetFilterLowpass(){
  return lp;
}

double SKFilter::GetFilterBandpass(){
  return bp;
}

double SKFilter::GetFilterHighpass(){
  return hp;
}

