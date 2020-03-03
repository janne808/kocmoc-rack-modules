/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Operator VCV Rack plugin.
 *
 *  Operator VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Operator VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Operator VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspphasorh__
#define __dspphasorh__

class Phasor{
public:
  // constructor/destructor
  Phasor(double initialPhase, double initialFrequency, double initialSampleRate);
  Phasor();
  ~Phasor();

  // state tick
  void Tick();
  
  // set parameters
  void SetPhase(double newPhase);
  void SetFrequency(double newFrequency);
  void SetPhaseModulation(double newPhaseModulation);
  void SetSampleRate(double newSampleRate);

  // get parameters
  double GetPhase();
  double GetPhaseIncrement();
  double GetFrequency();
  double GetPhaseModulation();
  double GetSampleRate();

private:
  double phase;
  double phaseIncrement;
  double phaseModulation;
  double frequency;
  double sampleRate;

  void ComputePhaseIncrement();
};

#endif
