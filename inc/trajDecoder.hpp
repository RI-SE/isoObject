#pragma once

#include <string>
#include <vector>

#include "iso22133.h"

/**
 * @brief Class for decoding TRAJ messages. Stores TRAJ data and
 * keeps track of unhandled bytes. 
 */
class TrajDecoder {
public: 
    TrajDecoder(bool debug) : debug(debug), expectingTRAJPoints(false) {};
    TrajDecoder() : debug(false), expectingTRAJPoints(false) {};
    ssize_t DecodeTRAJ(std::vector<char>*);
    bool ExpectingTrajPoints() const { return this->expectingTRAJPoints; }
    std::vector<TrajectorWaypointType> getTraj() const { 
        return this->trajectoryWaypoints; 
    }
    TrajectorHeaderType getTrajHeader() const { return this->trajecoryHeader; }
private:
    bool debug, expectingTRAJPoints;
    int nPointsHandled = 0;
    std::vector<char> unhandledBytes;
    std::vector<char> copiedData;
    std::vector<TrajectorWaypointType> trajectoryWaypoints;
    TrajectorHeaderType trajecoryHeader;
};


