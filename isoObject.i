/* File : isoObject.i */
%module isoObject
%{
#include "trajDecoder.hpp"
#include "iso22133state.hpp"
#include "iso22133object.hpp"
#include "socket.hpp"
#include "server.hpp"
#include "iso22133.h"
%}
%rename(BasicSocketEquals) operator=(BasicSocket&& other);
%rename(SocketEquals) operator=(Socket&& other);
%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);
%include "std_string.i"
%include "std_vector.i"
%include "stdint.i"
%include "cpointer.i"
%include "trajDecoder.hpp"
%include "iso22133state.hpp"
%include "iso22133object.hpp"
%include "socket.hpp"
%include "server.hpp"
%include "iso22133.h"
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
