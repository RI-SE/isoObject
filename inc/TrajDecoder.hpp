#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "iso22133.h"

namespace ISO22133 {

/**
 * @brief Class for decoding TRAJ messages. Stores TRAJ data and
 * keeps track of unhandled bytes. 
 */
class TrajDecoder {
public: 
    TrajDecoder(bool debug = false) : debug(debug), expectingTRAJPoints(false) {}
    ~TrajDecoder();
	ssize_t decode(const std::vector<char>& traj);
    bool expectingTrajPoints() const { return expectingTRAJPoints; }
	TrajectoryHeaderType getTrajHeader() const;
	std::vector<TrajectoryWaypointType> getTraj() const;

private:
    bool debug;
    bool expectingTRAJPoints;
    int nPointsHandled = 0;
    std::vector<char> unhandledBytes;
    std::vector<char> copiedData;
    std::vector<TrajectoryWaypointType> trajectoryWaypoints;
    TrajectoryHeaderType trajecoryHeader;
};

} // namespace ISO22133
