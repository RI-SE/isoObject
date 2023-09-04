/* File : isoObject.i */
%module(directors="1") isoObject_wrap
%feature("director") TestObject; 


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
%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);

%{
#include <boost/asio.hpp>
#include "trajDecoder.hpp"
#include "iso22133state.hpp"
#include "iso22133object.hpp"
#include "iso22133.h"
#include "udpServer.hpp"
#include "tcpServer.hpp"
%}

%include <boost/asio.hpp>
%include "trajDecoder.hpp"
%include "iso22133state.hpp"
%include "iso22133object.hpp"
%include "iso22133/iso22133.h"
%include "udpServer.hpp"
%include "tcpServer.hpp"


namespace std {                                                                  
    %template(TrajectoryWaypointVector) vector<TrajectoryWaypointType>;                                      
}; 

typedef double double_t;
typedef long int ssize_t;


struct timeval {
long int tv_sec;
long int tv_usec;
};


typedef struct {
	double longitudinal_m_s;
	double lateral_m_s;
	bool isLongitudinalValid;
	bool isLateralValid;
} SpeedType;

typedef struct {
    double xCoord_m;
    double yCoord_m;
    double zCoord_m;
    double heading_rad;
    bool isPositionValid;
    bool isHeadingValid;
    bool isXcoordValid;
    bool isYcoordValid;
    bool isZcoordValid;
} CartesianPosition;

typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    bool isLatitudeValid;
    bool isLongitudeValid;
    bool isAltitudeValid;
} GeographicPositionType;



typedef struct {
	double longitudinal_m_s2;
	double lateral_m_s2;
	bool isLongitudinalValid;
	bool isLateralValid;
} AccelerationType;
