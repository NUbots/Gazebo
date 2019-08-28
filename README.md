# Gazebo
Gazebo code for interfacing with the NUbots system

# Setup
  1. Install [CMake](https://cmake.org/download/) (v3.12+)
  2. Install [Gazebo](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1) (tested with versions 9 and 10).

  3. Install [NUclear](https://github.com/fastcode/nuclear) 

  4. Clone this repo:

    $ git clone git@github.com:NUbots/Gazebo.git

  5. Install Plugin:

    $ cd Gazebo
    $ mkdir build
    $ cd build
    $ cmake .. -GNinja
    $ ninja install

  6. Run plugin with gazebo:

    $ gazebo --verbose plugins/environment.world

  You are now ready to simulate!
