# ISO22133 Object

This is a iso22133 test object template class. This is intended to be used by 
anyone who needs to implement a test object following the ISO22133 protocol.

The object contains all communication and message handling for ISO22133, meaning 
you only need to create a new object instance and the rest takes care of itself.

It supports swig, and can therefore be used in e.g. python and java. See the swig directory for a python example.

## Dependencies
Boost system program-options thread

## Building
Clone the repository
```
git clone git@github.com:RI-SE/isoObject.git
```
Pull the submodules:
```
cd isoObject && git submodule update --init --recursive
```
Build the project:
```
mkdir build && cd build && cmake .. && make
```
If swigging, replace above ```cmake ..``` command with ```cmake .. -DWITH_SWIG=ON -DSWIG_WITH_X=ON```

Where X is either ```PYTHON``` or ```JAVA```.

Now the dynamic library ```libISO_object.so``` and the demo binary application ```ISO_objectDemo``` should be built in the build directory

### Installing
System wide install:
```
sudo make install
```
Reconfigure linker run-time bindings:
```
sudo ldconfig
```

## Standalone swig for java

To swig the ISOobject to Java the following command in terminal:

```
swig -java -c++ -package com.isoObject isoObject.i
```

The generated files can then be included and used in a Java project. 

## Usage
Since this is an abstract base class the first step is to create a new class and 
inherit the TestObject class, like so:

```c++
class myObject : public ISO22133::TestObject
```

This new class **must** implement the pure virtual function `handleAbort()`.
This acts as a safety function and is intended to contain necessary actions to
perform whenever an abort is requested, e.g internal safe-stop of the object.
The minimal version needed will look something like this:

```c++
class myObject : public ISO22133::TestObject {
    void handleAbort() { /* do abort stuff */ }
}
```

When a new TestObject instance is created there will be four threads created in
the background doing respectively: 
* Receive and handle messages on TCP from test server
* Receive and handle messages on UDP from test server
* Periodically send MONR messages to the test server
* Check the duration between HEABs and quit if not recieved in time

The values populating the MONR message must be continously updated using the corresponding 
setters, all values are per default 0.

For each ISO22133 message there is a signal/callback function associated that is
called every time a message arrives. If your test object needs to do anything 
special at the reception of a certain message, override the corresponding virtual
function. Example: 

```c++
void myObject::onSTRT(StartMessageType&) override {
    /* do special start stuff */
}
```

## Trajectory
The trajectory received from the test server is decoded and stored using a 
separate class `TrajDecoder`. The trajectory can be extracted to a 
`std::vector<TrajectoryWaypointType>` using `TestObject::getTrajectory()`.

## States
The state machine is imlemented using another abstract base class containg the 
logic for handling the different events specified in the ISO22133 protocol. 
This also contains the rules for which states the test object are allowed to 
transition to, it is for example not possible to make your object jump directly
to *Running* from *Disarmed*.

Each individual state is based on this base class and can be modified and
overridden in the same manner as the `TestObject`. This is to give another way 
of changing the behaviour of a test object in a specific state. All states have 
a `onEnter` and a `onExit` function to be used when the object needs to perform
special actions on a state transition. An example is the state *Pre-Arming* 
which in the default implementation transitions directly to *Armed*. Consider an 
object needing to perform some preparations in this state, the user would then 
create another version of `PreArming` like this:

```c++
class myPreArming : public ISO22133::PreArming {
public:
    void onEnter(ISO22133::TestObject& obj) override{
        DoPreArmStuff(); // Do preparations ...
        try { //... Then transition to Armed
            this->handleEvent(obj, ISO22133::Events::N); 
        }
        catch(const std::runtime_error& e) {
            std::cerr << e.what() << '\n';
        }
    }
};
```
To use this in the `TestObject` override the corresponding state creation 
function in your new test object class:
```c++
ISO22133::PreArming* myObject::createPreArming() const override {
    return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
}
```
