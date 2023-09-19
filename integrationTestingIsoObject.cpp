
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




int main(int argc, char** argv ) {
    auto args = parseArguments(argc, argv);

    std::string ip = args["listen-ip"].as<std::string>();
	myObject obj(ip);

    // wait for onTraj
    while (obj.getTrajectory().size() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // sleep for a while to get all trajectory points
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    

    auto traj = obj.getTrajectory();
    auto startX = traj[0].pos.xCoord_m;
    auto endX = traj[traj.size()-1].pos.xCoord_m;
    auto startY = traj[0].pos.yCoord_m;
    auto endY = traj[traj.size()-1].pos.yCoord_m;
    auto startZ = traj[0].pos.zCoord_m;
    auto endZ = traj[traj.size()-1].pos.zCoord_m;
    auto startYaw = traj[0].pos.heading_rad;
    auto endYaw = traj[traj.size()-1].pos.heading_rad;

    bool sendMonr = true;
    while (sendMonr) {
        obj.setMonr(startX, startY, startZ, startYaw, 0.0, 0.0);
        if (obj.getCurrentStateName() == "Running") {
            for (const auto& t : traj) {
                obj.setMonr(t.pos.xCoord_m, t.pos.yCoord_m, t.pos.zCoord_m, t.pos.heading_rad, 0.0, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                sendMonr = false;
            }
        }
    }

    std::cout << "done\n";
}

