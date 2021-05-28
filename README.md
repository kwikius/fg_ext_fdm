Controlling FlightGear externally using C++
-------------------------------------------

[![Youtube - Control FlightGear externally from C++](http://img.youtube.com/vi/27LC2WRK0Rk/2.jpg)](http://www.youtube.com/watch?v=27LC2WRK0Rk "YouTube - Control FlightGear externally from C++")

Description
-----------
 Simple example of how to control Flightgear externally, using sockets. 

examples/net_fdm_in

 The aircraft is suspended in space.
 Use the joystick to pose the model in roll, pitch and yaw. <a href="#note1" id="note1back"><sup>1</sup></a> <br>
 [Watch on Youtube](https://www.youtube.com/watch?v=27LC2WRK0Rk)

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
   - Enter $(project)/scripts directory.
   - $< ./flightgear.sh  # to start Flightgear
   - Wait for Flighgear to start and aircraft is visible.
   - Make a new terminal and cd to $(project)/bin directory 
   - Turn on Taranis transmitter and plug into usb port ( or otherwise activate the joystick port on "/dev/input/js0").
   - $< ./main.exe  # to start communicating
   - Twiddling sticks should now cause  aircraft to change attitude

Issues/ToDo
-----------
  - You may need to change channels or directions of channels on your transmitter to get things working smoothly.
  - Allow the gps location in $(project)/scripts/flightgear.sh and src/main.cpp to your own area/scenery.
  - <a id="note1" href="#note1back">[1]</a>  Currently whether the joystick uses World Frame or ModelFrame is hard coded.
     Todo use command line args

Acknowledgements
---------------
This work is derived from [David Calkin's example in the FlightGear source code](https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/examples/netfdm/)
Also uses calculations from FlightGear and CRRCSim



