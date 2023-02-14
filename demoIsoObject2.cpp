
#include <chrono>

#include "Object.hpp"

class myObject : public ISO22133::Object {
};




int main(int c, char** argv ) {

	myObject obj;
    obj.run();
}

