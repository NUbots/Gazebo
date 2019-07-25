# Gazebo
Gazebo code for interfacing with the NUbots system

# Setup
  1. Install [Gazebo](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1) (tested with version 10).
  2. Clone this repo:

    $ git clone git@github.com:NUbots/Gazebo.git

  3. Install Plugin:

    $ cd Gazebo
    $ mkdir build
    $ cd build
    $ cmake .. -GNinja
    $ ninja install

  4. Run plugin with gazebo:

    $ gazebo --verbose plugins/environment.world

  You are now ready to simulate!
