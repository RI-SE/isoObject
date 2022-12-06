/* File : isoObject.i */
%module isoObject_wrap
#ifdef SWIGJAVA
%javaconst(0);
#endif
%pointer_functions(uint32_t, uint32ptr);
%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>
%include <cpointer.i>
%include <typemaps.i>
%rename(LessThan) operator<(const Transition &lhs, const Transition &rhs);

%{
#include "trajDecoder.hpp"
#include "iso22133state.hpp"
#include "iso22133object.hpp"
#include "udpServer.hpp"
#include "tcpServer.hpp"
#include "iso22133.h"
%}


%include "trajDecoder.hpp"
%include "iso22133state.hpp"
%include "iso22133object.hpp"
%include "udpServer.hpp"
%include "tcpServer.hpp"
%include "iso22133.h"


typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};

typedef struct {
    double xCoord_m;
    double yCoord_m;
    double zCoord_m;
    double heading_rad;
    bool isPositionValid;
    bool isHeadingValid;
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
	struct timeval relativeTime;
	CartesianPosition pos;
	SpeedType spd;
	AccelerationType acc;
	float_t curvature;
} TrajectoryWaypointType;



