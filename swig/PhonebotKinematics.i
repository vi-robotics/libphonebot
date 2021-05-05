%module phonebotkinematicsjni

%pragma(java) jniclasscode=%{ 
  static { 
    try { 
        System.loadLibrary("phonebotjava"); 
    } catch (UnsatisfiedLinkError e) { 
      System.err.println("Native code library failed to load. \n" + e); 
      System.exit(1); 
    } 
  } 
%} 

%include "typemaps.i"
%apply float *OUTPUT { float *h1, float *h2, float *h3, float *h4, float *ex, float *ey };

%include "../PhonebotKinematics.hpp"
%{
#include "PhonebotKinematics.hpp"
%}
