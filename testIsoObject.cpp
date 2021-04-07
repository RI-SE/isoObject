
#include <iostream>
#include <thread>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "iso22133object.hpp"

class myState : public ISO22133::Disarmed {
public:
	void executeBehaviour(ISO22133::TestObject&);
};

class myRunning : public ISO22133::Running {
public:
	void executeBehaviour(ISO22133::TestObject&) {}
};

class myDisarmed : public ISO22133::Disarmed {
public:
    void onEnter(ISO22133::TestObject&) override{
        std::cout << "Entering disarmed" << std::endl;
    }
};

class myPreArming : public ISO22133::PreArming {
public:
    void onEnter(ISO22133::TestObject& obj) override{
        std::cout << "Entering Pre-Arming" << std::endl;
        this->handleEvent(obj, ISO22133::Events::N);
    }
};

class myObject : public ISO22133::TestObject {
	void handleAbort() { std::cout << "Bromsa!" << std::endl;}

	ISO22133::Running* createRunning() override {
		return dynamic_cast<ISO22133::Running*>(new myRunning);
	}
    
   // ISO22133::Disarmed* createDisarmed() override {
	//	return dynamic_cast<ISO22133::Disarmed*>(new myDisarmed);
	//}

    ISO22133::PreArming* createPreArming() override {
		return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
	}
};




int main(int c, char** argv ) {

	myObject obj;

    while (true) {
        usleep(1000);

        while(obj.isUdpOk()){
            obj.sendMONR(1);
            usleep(1000);
        }
    }
    

    std::cout << "done\n";

}
