/* File : isoObject.i */
%module isoObject_wrap

%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>
%include <cpointer.i>
%include <typemaps.i>
%javaconst(1);

%{
#include "trajDecoder.hpp"
#include "iso22133state.hpp"
#include "iso22133object.hpp"
#include "socket.hpp"
#include "server.hpp"
#include "iso22133.h"
%}

%include "trajDecoder.hpp"
%include "iso22133state.hpp"
%include "iso22133object.hpp"
%include "socket.hpp"
%include "server.hpp"
%include "iso22133.h"

//%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);

typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};

%pointer_functions(uint32_t, uint32ptr);