#include <iostream>
#include <mutex>

#include "trajDecoder.hpp"
#include "iso22133.h"

ssize_t TrajDecoder::DecodeTRAJ(std::vector<char>& dataBuffer, bool debug) {
    std::lock_guard<std::mutex> lock(this->guard);
    copiedData = dataBuffer;
    int tmpByteCounter;
    // Decode TRAJ Header
    if(!expectingTRAJPoints) {
        std::cout << "Receiving TRAJ" << std::endl;
        tmpByteCounter = decodeTRAJMessageHeader(&this->trajecoryHeader, 
            copiedData.data(), copiedData.size(), debug);
        if(tmpByteCounter < 0) {
            throw std::invalid_argument("Error decoding TRAJ Header");	
        }
        // Remove header bytes
        copiedData.erase(copiedData.begin(), copiedData.begin()+tmpByteCounter);	
        // The rest will be TRAJ waypoints
        expectingTRAJPoints = true;
        trajectoryWaypoints.clear();
        trajectoryWaypoints.reserve(trajecoryHeader.nWaypoints);
    }
    else {
        // Insert previously not treated bytes
        copiedData.insert(copiedData.begin(), unhandledBytes.begin(), 
            unhandledBytes.end());
    }

    // Decode TRAJ waypoints
    int tmpSize;
    TrajectoryWaypointType waypoint;

    tmpSize = trajecoryHeader.nWaypoints - nPointsHandled;
    for(int i = 0; i < tmpSize; i++) {
        // Save the bytes remaining and return
        if(copiedData.size() < sizeof(TRAJPointType)) { 
            unhandledBytes.resize(copiedData.size());
            unhandledBytes = copiedData; 	
            break;
        }

        // We have enough bytes, go ahead and decode waypoint
        tmpByteCounter = 
            decodeTRAJMessagePoint(&waypoint, copiedData.data(), debug);
        if(tmpByteCounter < 0) {
            throw std::invalid_argument("Error decoding TRAJ Waypoint");
        }
        // Remove the decoded bytes 
        copiedData.erase(copiedData.begin(), copiedData.begin()+tmpByteCounter);	
        trajectoryWaypoints.push_back(waypoint);
        nPointsHandled += 1;
    }
    std::cout << "Handling TRAJ point, ignore decoding errors" << std::endl;	

    if(nPointsHandled == trajecoryHeader.nWaypoints) {
        std::cout << "TRAJ received; " << 
            trajecoryHeader.nWaypoints << " points." << std::endl;
        expectingTRAJPoints = false; // Complete TRAJ received
        nPointsHandled = 0; // reset
        unhandledBytes.clear();
    }

    // Always return the complete buffer size since we don't want
    // the same bytes in here again
	return static_cast<int>(dataBuffer.size());
}

TrajectoryHeaderType TrajDecoder::getTrajHeader() const {
    std::lock_guard<std::mutex> lock(this->guard);
    return this->trajecoryHeader;
}

std::vector<TrajectoryWaypointType> TrajDecoder::getTraj() const {
    std::lock_guard<std::mutex> lock(this->guard);
    return this->trajectoryWaypoints; 
}
