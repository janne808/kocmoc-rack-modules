/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
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
#include "svfilter.h"
#include "iir.h"
#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 32

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.9

// constructor
SVFilter::SVFilter(double newCutoff, double newResonance, int newOversamplingFactor,
		   SVFFilterMode newFilterMode, double newSampleRate, SVFIntegrationMethod newIntegrationMethod){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  oversamplingFactor = newOversamplingFactor;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;

  SetFilterIntegrationRate();

  // initialize filter state
  hp = bp = lp = out = u_t1 = 0.0;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default constructor
SVFilter::SVFilter(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  oversamplingFactor = 2;
  filterMode = SVF_LOWPASS_MODE;
  sampleRate = 44100.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  hp = bp = lp = out = u_t1 = 0.0;
  
  integrationMethod = SVF_TRAPEZOIDAL;
  
  // instantiate downsampling filter
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0, IIR_DOWNSAMPLE_ORDER);
}

// default destructor
SVFilter::~SVFilter(){
  delete iir;
}

void SVFilter::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;

  SetFilterIntegrationRate();
  
  // initialize filter state
  hp = bp = lp = out = u_t1 = 0.0;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);
}

void SVFilter::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void SVFilter::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void SVFilter::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void SVFilter::SetFilterMode(SVFFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void SVFilter::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate/2.0);

  SetFilterIntegrationRate();
}

void SVFilter::SetFilterIntegrationMethod(SVFIntegrationMethod method){
  integrationMethod = method;
  ResetFilterState();
}

void SVFilter::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0 / (sampleRate * (double)(oversamplingFactor)) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt=0.0;
  }
}

double SVFilter::GetFilterCutoff(){
  return cutoffFrequency;
}

double SVFilter::GetFilterResonance(){
  return Resonance;
}

int SVFilter::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

double SVFilter::GetFilterOutput(){
  return out;
}

SVFFilterMode SVFilter::GetFilterMode(){
  return filterMode;
}

double SVFilter::GetFilterSampleRate(){
  return sampleRate;
}

SVFIntegrationMethod SVFilter::GetFilterIntegrationMethod(){
  return integrationMethod;
}

void SVFilter::filter(double input){
  // noise term
  double noise;

  // feedback amount variables
  double fb = 1.0 - (3.5*Resonance);

  // integration rate
  double dt2 = dt;
  
  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;

  // clamp integration rate
  switch(integrationMethod){
  case SVF_TRAPEZOIDAL:
    if(dt2 > 0.65){
      dt2 = 0.65;
    }
    break;
  case SVF_INV_TRAPEZOIDAL:
    if(dt2 > 1.0){
      dt2 = 1.0;
    }
    break;
  default:
    if(dt2 > 0.25){
      dt2 = 0.25;
    }
    break;
  }
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case SVF_SEMI_IMPLICIT_EULER:
      {
	// loss factor
	double beta = 1.0 - (0.0075/oversamplingFactor);

       	hp = input - lp - fb*bp - SinhPade54(bp);
	bp += dt2*hp;
	bp *= beta;
	lp += dt2*bp;
      }
      break;
    case SVF_TRAPEZOIDAL:
      // trapezoidal integration
      {
	double alpha = dt2/2.0;
	double beta = 1.0 - (0.0075/oversamplingFactor);
	double alpha2 = dt2*dt2/4.0 + fb*alpha;
	double D_t = (1.0 - dt2*dt2/4.0)*bp +
	              alpha*(u_t1 + input - 2.0*lp - fb*bp - SinhPade54(bp));
	double x_k, x_k2;

	// starting point is last output
	x_k = bp;
	
	// newton-raphson
	for(int ii=0; ii < 16; ii++) {
	  x_k2 = x_k - (x_k + alpha*SinhPade54(x_k) + alpha2*x_k - D_t)/
	                  (1.0 + alpha*CoshPade54(x_k) + alpha2);
	  
	  // breaking limit
	  if(abs(x_k2 - x_k) < 1.0e-9) {
	    x_k = x_k2;
	    break;
	  }
	  
	  x_k = x_k2;
	}

	lp += alpha*bp;
	bp = beta*x_k;
	lp += alpha*bp;
      	hp = input - lp - fb*bp;
      }
      break;
    case SVF_INV_TRAPEZOIDAL:
      // inverse trapezoidal integration
      {
	double alpha = dt2/2.0;
	double beta = 1.0 - (0.0075/oversamplingFactor);
	double alpha2 = dt2*dt2/4.0 + fb*alpha;
	double D_t = (1.0 - dt2*dt2/4.0)*bp +
	              alpha*(u_t1 + input - 2.0*lp - fb*bp - sinh(bp));
	double y_k, y_k2;

	// starting point is last output
	y_k = sinh(bp);
	
	// newton-raphson
	for(int ii=0; ii < 16; ii++) {
	  y_k2 = y_k - (alpha*y_k + ASinhPade54(y_k)*(1.0 + alpha2) - D_t)/
	                  (alpha + (1.0 + alpha2)*dASinhPade54(y_k));
	  
	  // breaking limit
	  if(abs(y_k2 - y_k) < 1.0e-9) {
	    y_k = y_k2;
	    break;
	  }
	  
	  y_k = y_k2;
	}

     	lp += alpha*bp;
	bp = beta*asinh(y_k);
	lp += alpha*bp;
      	hp = input - lp - fb*bp;
      }
      break;
    default:
      break;
    }
    
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
      out = iir->IIRfilter(out);
    }
  }
  
  // set input at t-1
  u_t1 = input;    
}

double SVFilter::GetFilterLowpass(){
  return lp;
}

double SVFilter::GetFilterBandpass(){
  return bp;
}

double SVFilter::GetFilterHighpass(){
  return hp;
}

