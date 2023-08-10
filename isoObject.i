/* File : isoObject.i */
%module(directors="1") isoObject_wrap
%feature("director") TestObject; 

%begin %{
#define SWIG_PYTHON_STRICT_BYTE_CHAR
%}


#ifdef SWIGJAVA
%javaconst(0);
#endif

#ifdef SWIGPYTHON
%rename(timevalLessThan) operator< (const timeval &lhs, const timeval &rhs);
%rename(timevalGreaterThan) operator> (const timeval &lhs, const timeval &rhs);
#endif

%pointer_functions(uint32_t, uint32ptr);
%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>
%include <cpointer.i>
%include <typemaps.i>
%include <pybuffer.i>
%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);
// change "(const char* data, int len)" to match your functions declaration
%apply (char *STRING, size_t LENGTH) { (const char* data, int len) }

%{
#include <boost/asio.hpp>
#include "trajDecoder.hpp"
#include "iso22133state.hpp"
#include "iso22133object.hpp"
#include "iso22133.h"
#include "udpServer.hpp"
#include "tcpServer.hpp"
#include "positioning.h"
%}

%include <boost/asio.hpp>
%include "trajDecoder.hpp"
%include "iso22133state.hpp"
%include "iso22133object.hpp"
%include "iso22133/iso22133.h"
%include "udpServer.hpp"
%include "tcpServer.hpp"
%include "positioning.h"


namespace std {                                                                  
    %template(TrajectoryWaypointVector) vector<TrajectoryWaypointType>;
}; 

typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};