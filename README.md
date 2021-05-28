Controlling FlightGear externally using C++
-------------------------------------------

[![Youtube - Control FlightGear externally from C++](http://img.youtube.com/vi/27LC2WRK0Rk/2.jpg)](http://www.youtube.com/watch?v=27LC2WRK0Rk "YouTube - Control FlightGear externally from C++")

Description
-----------
 Some examples of how to control Flightgear externally, using sockets.

Examples
--------
  * examples/net_fdm_out.
     The aircraft is suspended in space.
     Use the joystick to pose the model in roll, pitch and yaw. <br> 
     <a href="#note1" id="note1back"><sup>1</sup></a> Choose either euler angles or quaternions:
     [Watch on Youtube](https://www.youtube.com/watch?v=27LC2WRK0Rk)

  * examples/net_fdm_in.
    Retrieves the net_fdm structure from Flightgear. Displays current pitch roll and yaw values. N.B. Plane is controlled by Joystick internally from Flightgear

  * examples/telnet.
    Read and write variables to/from FlightGear telnet interface. Sends joystick values to control aircraft using telnet protocol.

  * examples/io.
    Sends control values (roll, pitch, yaw) in to control the aircraft and reads the FlightGear fdm structure, the basic requirements for SITL.
    Control is from joystick but can quite easily be injected from another source such as an autopilot or flightcontroller.
    Similarly the outputs of the fdm structure should be adequate to simulate sensors. 

  - <a id="note1" href="#note1back">[1]</a>   
    * $< net_fdm_out -r euler  # Map joystick to world coordinates using euler angles
    * $< net_fdm_out -r quat   # map the joystick to model frame using quaternion.

Requires
--------
 - Joystick on "/dev/input/js0". (For example Taranis or other OpenTX transmitter with usb connection).
 - [FlightGear](https://www.flightgear.org/).
 - [quan library](https://github.com/kwikius/quan-trunk).
 - gcc7 or higher (uses c++14)

Build
-----
   - Make a new  terminal 
   - Make a $(project) directory and enter the $(project) directory
   - $< git clone https://github.com/kwikius/quan-trunk.git ( or [download zip](https://github.com/kwikius/quan-trunk/archive/refs/heads/master.zip) and unzip)
   - $< export QUAN_ROOT=~/my/path/to/quan-trunk
   - $< make

Run
---
   - Enter $(project)/examples/\<example\> directory.
   - Turn on Taranis transmitter and plug into usb port ( or otherwise activate the joystick port on "/dev/input/js0").
   - $< ./flightgear.sh  # to start Flightgear
   - Wait for Flightgear to start and aircraft is visible.
   - Make a new terminal and cd to $(project)/bin directory 
   - $< ./bin/\<example\>.exe  # to start communicating
   - Twiddling sticks should now control the sim.

Issues/ToDo
-----------
  - You may need to change channels or directions of channels on your transmitter to get things working smoothly.
  - Allow the gps location in $(project)/scripts/flightgear.sh and src/main.cpp to your own area/scenery.


Acknowledgements
---------------
This work is derived from [David Calkin's example in the FlightGear source code](https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/examples/netfdm/)
Also uses calculations from FlightGear and CRRCSim



