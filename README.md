Gui overlay plugin for Gazebo. This plugin allows you to update the value of PID controllers in your model.

How To Use:

* Requires Cmake, g++
* mkdir build && cd build
* cmake ..
* make
* Then run Gazebo with example world. Insert your model into Gazebo, you can now select the various joints and update their PID values.
* Only works for joints that have non-default controllers attached to them. I.E to P, I, D values are not the default and controller actually has affect.
