%module pidcontrollerjni

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

%include "../PIDController.hpp"
%{
#include "PIDController.hpp"
%}
