#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "iso22133.h"
#include "traj.h"

/**
 * @brief Class for decoding TRAJ messages. Stores TRAJ data and
 * keeps track of unhandled bytes. 
 */
class TrajDecoder {
public: 
    TrajDecoder(bool debug) : debug(debug), expectingTRAJPoints(false) {};
    TrajDecoder() : debug(false), expectingTRAJPoints(false) {};
    ssize_t DecodeTRAJ(std::vector<char>&, bool debug = false);
    bool ExpectingTrajPoints() const { return this->expectingTRAJPoints; }
    TrajectoryHeaderType getTrajHeader() const;
    std::vector<TrajectoryWaypointType> getTraj() const;

private:
    mutable std::mutex guard;
    bool debug;
    std::atomic<bool> expectingTRAJPoints;
    int nPointsHandled = 0;
    std::vector<char> unhandledBytes;
    std::vector<char> copiedData;
    std::vector<TrajectoryWaypointType> trajectoryWaypoints;
    TrajectoryHeaderType trajecoryHeader;
};


