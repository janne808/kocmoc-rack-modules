#ifndef __kocmocfastmathh__
#define __kocmocfastmathh__

// pade 9/8 approximant for sinh
inline double SinhPade98(double x) {
  // return approximant
  return (x*(4585922449.0*x*x*x*x*x*x*x*x + 1066023933480.0*x*x*x*x*x*x + 83284044283440.0*x*x*x*x + 2303682236856000.0*x*x + 15605159573203200.0))/
            (45.0*(1029037.0*x*x*x*x*x*x*x*x - 345207016.0*x*x*x*x*x*x + 61570292784.0*x*x*x*x - 6603948711360.0*x*x + 346781323848960.0));
}

// pade 9/8 approximant for asinh
inline double ASinhPade98(double x) {
  // return approximant
  return (x*(4474275508260072601.0*x*x*x*x*x*x*x*x + 152904157921385089560.0*x*x*x*x*x*x + 876802506140506785840.0*x*x*x*x + 1599149222427667310400.0*x*x + 900717260398840684800.0))/
    (315.0*(42981288509837475.0*x*x*x*x*x*x*x*x + 779000561224162200.0*x*x*x*x*x*x + 3494582558460865872.0*x*x*x*x + 5553234177230076480.0*x*x + 2859419874282033920.0));
}

// pade 5/4 approximant for sinh
inline double SinhPade54(double x) {
  // return approximant
  return (x*(551.0*x*x*x*x + 22260*x*x + 166320.0))/(15.0*(5.0*x*x*x*x - 364.0*x*x + 11088.0));
}

// pade 5/4 approximant for asinh
inline double ASinhPade54(double x) {
  // return approximant
  return (x*(69049.0*x*x*x*x + 717780.0*x*x + 922320.0))/(15.0*(9675.0*x*x*x*x + 58100.0*x*x + 61488.0));
}

// pade 5/4 approximant for derivative of asinh
inline double dASinhPade54(double x) {
  double n = 44536605.0*x*x*x*x*x*x*x*x + 339381280.0*x*x*x*x*x*x + 2410740304.0*x*x*x*x + 5254518528.0*x*x + 3780774144.0;
  double d = 9675.0*x*x*x*x + 58100.0*x*x + 61488.0;

  // return approximant
  return n/(d*d);
}

// pade 3/2 approximant for sinh
inline double SinhPade32(double x) {
  // return approximant
  return -(x*(7.0*x*x + 60.0))/(3.0*(x*x - 20.0));
}

// pade 3/4 approximant for sinh
inline double SinhPade34(double x) {
  // return approximant
  return (20.0*x*(31.0*x*x*x*x + 294.0))/(11.0*x*x*x*x - 360.0*x*x + 5880.0);
}

// pade 3/2 approximant for cosh
inline double CoshPade32(double x) {
  // return approximant
  return -(5.0*x*x + 12.0)/(x*x - 12.0);
}

// pade 3/4 approximant for cosh
inline double CoshPade34(double x) {
  // return approximant
  return (4.0*(61.0*x*x + 150.0))/(3.0*x*x*x*x - 56.0*x*x + 600.0);
}

// pade 5/4 approximant for cosh
inline double CoshPade54(double x) {
  // return approximant
  return (313.0*x*x*x*x + 6900.0*x*x + 15120.0)/(13.0*x*x*x*x - 660.0*x*x + 15120.0);
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
  return x*(15.0 + x*x)/(15.0 + 6.0*x*x);
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
  return x*(945.0 + 105.0*x*x+x*x*x*x)/(945.0 + 420.0*x*x + 15.0*x*x*x*x);
}

inline double SinhExpTaylor(double x, int N) {
  double n=1.0, d=1.0, s=-1.0, t=1.0, exp_plus=1.0, exp_minus=1.0;
  
  // compute power series approximation
  for(int ii=2; ii < N; ii++){
    n *= x;
    t = n/d;
    exp_plus += t;
    exp_minus += s*t;
    d *= ii;
    s *= -1.0;
  }

  return (exp_plus - exp_minus)/2.0;
}

inline double ExpTaylor(double x, int N) {
  double n=1.0, d=1.0, t=1.0, exp=1.0;
  
  // compute power series approximation
  for(int ii=2; ii < N; ii++){
    n *= x;
    t = n/d;
    exp += t;
    d *= ii;
  }

  return exp;
}

inline double BramSaturator(double x, double a) {
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

inline double TanhExpTaylor(double x, int N) {
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

#endif
