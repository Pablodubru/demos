# demos

Executable naming convention:
controlscheme-controller-controlledvariable-motiontype

Name descriptions:
Schemes:
*imufb: feedback from imu sensory data.

Controllers:
*fo: Fractional order controller.
*io: Integer order controller.

Variables:
*inc: Inclination position control.

Motion types:
*yes: Two inclinations forward.

Current working demos:
* imufb-inc-fo-yes
* encoderfb-fo-yes.cpp
* ol-pos-360
* clInc20degstep


# dependencies
Install fftw, eigen and qt5 serial port libraries in debian-based with:

``
sudo apt install libfftw3-dev libeigen3-dev libqt5serialport5-dev libplot-dev
``

Add user to dialup group with:

``
sudo usermod -a -G dialout <user>
``

and reboot.
