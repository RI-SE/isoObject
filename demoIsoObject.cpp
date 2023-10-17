
#include <chrono>


#include "iso22133object.hpp"
#include "printUtil.hpp"
#include <boost/program_options.hpp>

namespace po = boost::program_options;
/*!
 * \brief Parse the command line arguments. If the user has specified the
 *        --help option, print the help message and exit.
 * \param argc The number of arguments.
 * \param argv The arguments.
 * \return The parsed arguments.
 */
static po::variables_map parseArguments(
	int argc,
	char** argv)
{
	po::variables_map ret;
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "print this message")
		("listen-ip,i", po::value<std::string>()->default_value("0.0.0.0"), "The IP address that the isoObject will listen on.")
        ("behaviour,b", po::value<std::string>()->default_value("follow-trajectory"), "The behaviour of the isoObject. Options are 'follow-trajectory', 'dynamic', and 'circle'")
	;
	po::store(po::parse_command_line(argc, argv, desc), ret);
	po::notify(ret);
	if (ret.count("help")){
		std::cout << desc << std::endl;
		exit(EXIT_FAILURE);
	}
	return ret;
}

class myDisarmed : public ISO22133::Disarmed {
public:
    /**
     * @brief Called once when entering state
     * 
     * @param obj 
     */
    void onEnter(ISO22133::TestObject& obj) override {
        std::cout << "Entering disarmed" << std::endl;
    }

    /**
     * @brief Called once when leaving state
     * 
     */
    void onExit(ISO22133::TestObject&) override{
        std::cout << "Leaving disarmed" << std::endl;
    }

};

class myPreArming : public ISO22133::PreArming {
public:
    void onEnter(ISO22133::TestObject& obj) override{
        std::cout << "Entering Pre-Arming" << std::endl;
        try {
            this->handleEvent(obj, ISO22133::Events::N);
        }
        catch(const std::runtime_error& e) {
            std::cerr << e.what() << '\n';
        }
    }
};



class myObject : public ISO22133::TestObject {
public:
    std::vector<TrajectoryWaypointType> trajectory;

    void setMonr(double x,
                 double y, 
                 double z, 
                 double heading_rad, 
                 double lateral_m_s, 
                 double lonitudinal_m_s) {
        // Initialize required fields in MONR
        CartesianPosition pos;
        SpeedType spd;
        pos.xCoord_m = x;
        pos.yCoord_m = y;
        pos.zCoord_m = z;
        pos.heading_rad = heading_rad;
        pos.isHeadingValid = true;
        pos.isPositionValid = true;
        pos.isXcoordValid = true;
        pos.isYcoordValid = true;
        pos.isZcoordValid = true;
        spd.lateral_m_s = lateral_m_s;
        spd.longitudinal_m_s = lonitudinal_m_s;
        spd.isLateralValid = true;
        spd.isLongitudinalValid = true;

        this->setPosition(pos);
        this->setSpeed(spd);
    }
    myObject(std::string ip) : 
                ISO22133::TestObject(ip), 
                dummyMember(0) {
        ObjectSettingsType osem; 
        osem.testMode = TEST_MODE_UNAVAILABLE;
        setMonr(1,2,3,0.4,5,6);
        setObjectSettings(osem);
    }
    /**
     * @brief User must override this function for handling internal
     * abort prerequisites of the test object
     * 
     */
	void handleAbort() { std::cout << "Bromsa!" << std::endl;}
    
    /**
     * @brief Create a myDisarmed object. 
     * This is an example of how to override the state creation 
     * functions to get the new state
     * 
     * @return ISO22133::Disarmed* 
     */
    ISO22133::Disarmed* createDisarmed() const override {
		return dynamic_cast<ISO22133::Disarmed*>(new myDisarmed);
	}

    ISO22133::PreArming* createPreArming() const override {
		return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
	}

    //! overridden on*message* function.
    void onOSEM(ObjectSettingsType& osem) override {
        std::cout << "Object Settings Received" << std::endl;
        setObjectSettings(osem);
        PRINT_STRUCT(ObjectSettingsType, &osem,
            PRINT_FIELD(TestModeType, testMode)
        )

    }

    void onTRAJ() override {
        std::cout << "Got onTRAJ signal, fetching new traj segments" << std::endl;
        std::vector<TrajectoryWaypointType> newTraj;
        newTraj = this->getTrajectory();
        if (this->getObjectSettings().testMode == TEST_MODE_ONLINE){
            std::cout << "Test mode is online planned, appending new trajectory to existing" << std::endl;
            this->trajectory.insert(this->trajectory.end(), newTraj.begin(), newTraj.end());

            // We might receive trajectories that overlap, we remove the duplicate points by checking the time
            std::sort(this->trajectory.begin(), this->trajectory.end(), [](const TrajectoryWaypointType& t1, const TrajectoryWaypointType& t2) {
                return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec < t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec;
            });
            this->trajectory.erase(std::unique(this->trajectory.begin(), this->trajectory.end(), [](const TrajectoryWaypointType& t1, const TrajectoryWaypointType& t2) {
                return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec == t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec;
            }), this->trajectory.end());
        } else {
            std::cout << "Test mode is preplanned, replacing existing trajectory" << std::endl;
            this->trajectory = newTraj;
        }
        std::cout << "Trajectory size: " << this->trajectory.size() << std::endl;
    }

    void onSTRT(StartMessageType&) override {
        std::cout << "Object Starting" << std::endl;
    }

    //! overridden vendor specific message handling
    int handleVendorSpecificMessage(const int msgType, const std::vector<char>& data) override {
        int handledBytes = 0;
        RemoteControlManoeuvreMessageType DCMMmsg;
        switch (msgType)
        {
        case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM:
            handledBytes = decodeDCMMMessage(data.data(), data.size(), &DCMMmsg, 0);
            if(handledBytes < 0) {
                throw std::invalid_argument("Error decoding DCMM");
            }
            else {
                std::cout << "Handled DCMM Message" << std::endl;
            }
            break;
        
        default:
            break;
        }

        return handledBytes;
    }
private:
    int dummyMember;
    void dummyFunc() {
        std::stringstream ss;
        ss << "I am printed in a useless function" << std::endl;
        std::cout << ss.str();
    };

};

// Function that can parse both number and dot notation and hostnames into IP addresses
std::string resolveIP (std::string listen_ip) {
    addrinfo hints = {0};
    addrinfo* result;
    in_addr_t ip;
    hints.ai_family = AF_INET; // Use AF_INET6 for IPv6
    hints.ai_socktype = SOCK_STREAM;

    int status = getaddrinfo(listen_ip.c_str(), nullptr, &hints, &result);
    if (status != 0) {
        std::cout << "Failed to resolve address for value %s, Default to 0.0.0.0" << listen_ip << std::endl;
        return "0.0.0.0";
    }

    ip = ((sockaddr_in*)result->ai_addr)->sin_addr.s_addr;
    freeaddrinfo(result);

    // Convert binary IP to string for logging
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &ip, ip_str, INET_ADDRSTRLEN);
    return ip_str;
}

/**
 * @brief  ISO-object that automatically gets all the points from the trajectory when connected,
 * and will set its location to the first point of the trajectory when armed. It will then follow
 * the trajectory when running and set its location to the last point when done.
 * 
 */
void runFollowTrajectory(myObject& obj) {
    std::vector<TrajectoryWaypointType> traj;
    double startX;
    double endX;
    double startY;
    double endY;
    double startZ;
    double endZ;
    double startYaw;
    double endYaw;

    auto finishedRunning = false;
    while(1) {
        auto state = obj.getCurrentStateName();
        if (state == "Disarmed") {
            // sleep for a while to get all trajectory points
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            traj = obj.getTrajectory();
            startX = traj[0].pos.xCoord_m;
            endX = traj.back().pos.xCoord_m;
            startY = traj[0].pos.yCoord_m;
            endY = traj.back().pos.yCoord_m;
            startZ = traj[0].pos.zCoord_m;
            endZ = traj.back().pos.zCoord_m;
            startYaw = traj[0].pos.heading_rad;
            endYaw = traj.back().pos.heading_rad;
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
            finishedRunning = false;
        }
        else if (state == "Armed") {
            obj.setMonr(startX, startY, startZ, startYaw, 0.0, 0.0);
        }
        else if (finishedRunning) {
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
        }
        else if (state == "Running") {
            for (int i = 0; i < traj.size() - 1; ++i) {
                auto currentTraj = traj[i];
                auto nextTraj = traj[i+1];
                auto secondsDiff = nextTraj.relativeTime.tv_sec - currentTraj.relativeTime.tv_sec;
                auto microsecondsDiff = nextTraj.relativeTime.tv_usec - currentTraj.relativeTime.tv_usec;
                auto timeDiff = secondsDiff * 1000000 + microsecondsDiff;

                obj.setMonr(currentTraj.pos.xCoord_m, currentTraj.pos.yCoord_m, currentTraj.pos.zCoord_m, currentTraj.pos.heading_rad, currentTraj.spd.lateral_m_s, currentTraj.spd.longitudinal_m_s);
                std::this_thread::sleep_for(std::chrono::microseconds(timeDiff));
            }
            finishedRunning = true;
        }
    }
}


/**
 * @brief ISO-object that can be used with dynamic trajectories. The ISO-object works in the same way as runFollowTrajectory, but it will get
 * a new trajectory at runtime, instead of the full trajectory when connecting.
 * 
 */
void runDynamic(myObject& obj) {
    std::vector<TrajectoryWaypointType> traj;
    double startX;
    double endX;
    double startY;
    double endY;
    double startZ;
    double endZ;
    double startYaw;
    double endYaw;

    auto finishedRunning = false;
    while(1) {
        auto state = obj.getCurrentStateName();
        if (state == "Disarmed") {
            // sleep for a while to get all trajectory points
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            traj = obj.getTrajectory();
            startX = traj[0].pos.xCoord_m;
            endX = traj.back().pos.xCoord_m;
            startY = traj[0].pos.yCoord_m;
            endY = traj.back().pos.yCoord_m;
            startZ = traj[0].pos.zCoord_m;
            endZ = traj.back().pos.zCoord_m;
            startYaw = traj[0].pos.heading_rad;
            endYaw = traj.back().pos.heading_rad;
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
            finishedRunning = false;
        }
        else if (state == "Armed") {
            obj.setMonr(startX, startY, startZ, startYaw, 0.0, 0.0);
        }
        else if (finishedRunning) {
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
        }
        else if (state == "Running") {
            int i = 0;
            TrajectoryWaypointType currentTraj;
            TrajectoryWaypointType nextTraj;
            while (i < traj.size()) {
                currentTraj = traj[i];
                nextTraj = traj[i+1];
                auto secondsDiff = nextTraj.relativeTime.tv_sec - currentTraj.relativeTime.tv_sec;
                auto microsecondsDiff = nextTraj.relativeTime.tv_usec - currentTraj.relativeTime.tv_usec;
                auto timeDiff = secondsDiff * 1000000 + microsecondsDiff;

                obj.setMonr(currentTraj.pos.xCoord_m, currentTraj.pos.yCoord_m, currentTraj.pos.zCoord_m, currentTraj.pos.heading_rad, currentTraj.spd.lateral_m_s, currentTraj.spd.longitudinal_m_s);
                std::this_thread::sleep_for(std::chrono::microseconds(timeDiff));
                ++i;
                traj = obj.trajectory;
            }
            endX = currentTraj.pos.xCoord_m;
            endY = currentTraj.pos.yCoord_m;
            endZ = currentTraj.pos.zCoord_m;
            endYaw = currentTraj.pos.heading_rad;
            finishedRunning = true;
        }
    }
}

/**
 * @brief ISO-object that moves in a circle when connected.
 * 
 */
void runCircle(myObject& obj) {
    double originX = 0.0;
    double originY = 0.0;
    double originZ = 0.0;
    double radius = 5.0;
    double angle = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        angle += 0.005;
        if (angle > 2 * M_PI) {
            angle = 0.0;
        }
        x = originX + radius * cos(angle);
        y = originY + radius * sin(angle);
        z = originZ + radius/2 * sin(angle);
        if (z < 0) {
            z = 0;
        }
        // Todo calculate heading and speed
        obj.setMonr(x, y, z, angle + M_PI / 2, 0.0, 0.0);
    }
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv ) {
    auto args = parseArguments(argc, argv);
    auto ip = resolveIP(args["listen-ip"].as<std::string>());
    myObject obj(ip);
    std::string behaviour = args["behaviour"].as<std::string>();
    if (behaviour == "follow-trajectory") {
        runFollowTrajectory(obj);
    }
    else if (behaviour == "dynamic") {
        runDynamic(obj);
    }
    else if (behaviour == "circle") {
        runCircle(obj);
    }
    else {
        std::invalid_argument("Unknown behaviour");
    }
}

