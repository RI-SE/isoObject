
#include <chrono>


#include "iso22133object.hpp"


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
    myObject() : dummyMember(0) {
        // Initialize required fields in MONR
        CartesianPosition pos;
        SpeedType spd;
        pos.xCoord_m = 0;
        pos.yCoord_m = 0;
        pos.zCoord_m = 0;
        pos.heading_rad = 0;
        pos.isHeadingValid = true;
        pos.isPositionValid = true;
        spd.lateral_m_s = 0;
        spd.longitudinal_m_s = 0;
        spd.isLateralValid = true;
        spd.isLongitudinalValid = true;

        this->setPosition(pos);
        this->setSpeed(spd);
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
    void onOSEM(ObjectSettingsType&) override {
        std::cout << "overridden onOSEM, inc private member" << std::endl;
        dummyMember++;
        std::cout << "dummyMember is now " << dummyMember << std::endl;
        std::cout << "We can now also add arbitrary functions: " << std::endl;
        dummyFunc();

    }

    void onSTRT(StartMessageType&) override {
        std::cout << "Object Starting" << std::endl;
    }
private:
    int dummyMember;
    void dummyFunc() {
        std::cout << "I am printed in a useless function" << std::endl;
    };

};




int main(int c, char** argv ) {

	myObject obj;
    
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  
    }
    
    std::cout << "done\n";
}

