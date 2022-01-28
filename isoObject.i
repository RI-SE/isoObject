/* File : isoObject.i */
%module isoObject

%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);
%include "std_string.i"
%include "stdint.i"
%include "cpointer.i"
%include "../../../iso22133/iso22133.h"
%include "../../../iso22133/positioning.h"
%include "/inc/trajDecoder.hpp"
%include "/inc/iso22133state.hpp"
%include "/inc/iso22133object.hpp"
%include "typemaps.i"
%javaconst(1);

typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};

%pointer_functions(uint32_t, uint32ptr);

//swig -java -c++ -package com.isoObject isoObject.i

//gcc -fPIC -Wall -c ../../../iso22133/iso22133.h ../../../iso22133/positioning.h isoObject_wrap.cxx -I/usr/lib/jvm/jdk-15.0.2/include/ -I/usr/lib/jvm/jdk-15.0.2/include/linux/
