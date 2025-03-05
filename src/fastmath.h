/*
 *  (C) 2025 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Kocmoc VCV Rack module.
 *
 *  Kocmoc VCV Rack module is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Kocmoc VCV Rack module is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Kocmoc VCV Rack module.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __kocmocfastmathh__
#define __kocmocfastmathh__

// pade 3/2 approximant for sinh
inline float FloatSinhPade32(float x) {
  // return approximant
  return -1.0f * (x * (7.0f * x * x + 60.0f)) / (3.0f * (x * x - 20.0f));
}

// pade 3/2 approximant for sinh
inline double SinhPade32(double x) {
  // return approximant
  return -1.0 * (x * (7.0 * x * x + 60.0)) / (3.0 * (x * x - 20.0));
}

// pade 3/4 approximant for sinh
inline double SinhPade34(double x) {
  // return approximant
  return (20.0 * x * (31.0 * x * x * x * x + 294.0)) /
           (11.0 * x * x * x * x - 360.0 * x * x + 5880.0);
}

// pade 5/4 approximant for sinh
inline float FloatSinhPade54(float x) {
  // return approximant
  return (x * (x * x * (551.0f * x * x + 22260.0f) + 166320.0f)) / (15.0f * (x * x * (5.0f * x * x - 364.0f) + 11088.0f));
}

// pade 5/4 approximant for sinh
inline double SinhPade54(double x) {
  // return approximant
  return (x * (551.0 * x * x * x * x + 22260*x*x + 166320.0))/(15.0 * (5.0 * x * x * x * x - 364.0 * x * x + 11088.0));
}

// pade 3/2 approximant for cosh
inline float FloatCoshPade32(float x) {
  // return approximant
  return -1.0f * (5.0f * x * x + 12.0f) / (x * x - 12.0f);
}

// pade 3/2 approximant for cosh
inline double CoshPade32(double x) {
  // return approximant
  return -1.0 * (5.0 * x * x + 12.0) / (x * x - 12.0);
}

// pade 3/4 approximant for cosh
inline double CoshPade34(double x) {
  // return approximant
  return (4.0 * (61.0 * x * x + 150.0)) / (3.0 * x * x * x * x - 56.0 * x * x + 600.0);
}

// pade 5/4 approximant for cosh
inline float FloatCoshPade54(float x) {
  // return approximant
  return (x * x * (313.0f * x * x + 6900.0f) + 15120.0f) /
           (x * x * (13.0f * x * x - 660.0f) + 15120.0f);
}

// pade 5/4 approximant for cosh
inline double CoshPade54(double x) {
  // return approximant
  return (313.0 * x * x * x * x + 6900.0 * x * x + 15120.0) /
           (13.0 * x * x * x * x - 660.0 * x * x + 15120.0);
}

// pade 5/4 approximant for asinh
inline float FloatASinhPade54(float x) {
  // return approximant
  return (x * (69049.0f * x * x * x * x + 717780.0f * x * x + 922320.0f)) /
           (15.0f * (9675.0f * x * x * x * x + 58100.0f * x * x + 61488.0f));
}

// pade 5/4 approximant for asinh
inline double ASinhPade54(double x) {
  // return approximant
  return (x * (69049.0 * x * x * x * x + 717780.0 * x * x + 922320.0)) /
           (15.0 * (9675.0 * x * x * x * x + 58100.0 * x * x + 61488.0));
}

// pade 5/4 approximant for derivative of asinh
inline float FloatdASinhPade54(float x) {
  float n = 44536605.0f * x * x * x * x * x * x * x * x + 339381280.0f * x * x * x * x * x * x +
              2410740304.0f * x * x * x * x + 5254518528.0f * x * x + 3780774144.0f;
  float d = 9675.0f * x * x * x * x + 58100.0f * x * x + 61488.0f;

  // return approximant
  return n / (d * d);
}

// pade 5/4 approximant for derivative of asinh
inline double dASinhPade54(double x) {
  double n = 44536605.0 * x * x * x * x * x * x * x * x + 339381280.0 * x * x * x * x * x * x +
               2410740304.0 * x * x * x * x + 5254518528.0 * x * x + 3780774144.0;
  double d = 9675.0 * x * x * x * x + 58100.0 * x * x + 61488.0;

  // return approximant
  return n / (d * d);
}

// pade 2/3 approximant for tanh
inline float FloatTanhPade23(float x) {
  // return approximant
  return (3.0f * x) / (x * x + 3.0f);
}

// pade 2/3 approximant for tanh
inline double TanhPade23(double x) {
  // return approximant
  return (3.0 * x) / (x * x + 3.0);
}

// pade 4/5 approximant for tanh
inline float FloatTanhPade45(float x) {
  // return approximant
  return (5.0f * x * (2.0f * x * x + 21.0f)) / (x * x * (x * x + 45.0f) + 105.0f);
}

// pade 4/5 approximant for tanh
inline double TanhPade45(double x) {
  // return approximant
  return (5.0 * x * (2.0 * x * x + 21.0))/(x * x * (x * x + 45.0) + 105.0);
}

// pade 3/2 approximant for tanh
inline double TanhPade32(double x) {
  // clamp x to -3..3
  if(x > 3.0) {
    x = 3.0;
  }
  else if(x < -3.0) {
    x = -3.0;
  }
  // return approximant
  return x * (15.0 + x * x) / (15.0 + 6.0 * x * x);
}

// pade 5/4 approximant for tanh
inline double TanhPade54(double x) {
  // clamp x to -4..4
  if(x > 4.0) {
    x = 4.0;
  }
  else if(x < -4.0) {
    x = -4.0;
  }
  // return approximant
  return x * (945.0 + 105.0 * x * x + x * x * x * x) /
           (945.0 + 420.0 * x * x + 15.0 * x * x * x * x);
}

inline double SinhExpTaylor(double x, int N) {
  double n=1.0, d=1.0, s=-1.0, t=1.0, exp_plus=1.0, exp_minus=1.0;
  
  // compute power series approximation
  for(int ii=2; ii < N; ii++){
    n *= x;
    t = n / d;
    exp_plus += t;
    exp_minus += s * t;
    d *= ii;
    s *= -1.0;
  }

  return (exp_plus - exp_minus) / 2.0;
}

inline double ExpTaylor(double x, int N) {
  double n=1.0, d=1.0, t=1.0, exp=1.0;
  
  // compute power series approximation
  for(int ii=2; ii < N; ii++){
    n *= x;
    t = n / d;
    exp += t;
    d *= ii;
  }

  return exp;
}

inline double TanhExpTaylor(double x, int N) {
  double e;

  // clamp x to -3..3
  if(x > 3.0) {
    x = 3.0;
  }
  else if(x < -3.0) {
    x = -3.0;
  }
  
  e = ExpTaylor(2.0 * x, N);
  
  return (e - 1.0) / (e + 1.0);
}

#endif
