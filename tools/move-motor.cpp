#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
    //--Can port communications--
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 20 );
    CiA402Device m (3, &pm1, &sd1);


    // motors must be turned ON

    double pos;
    double vel;

    m.Reset();
    m.SwitchOn();
    m.Setup_Velocity_Mode();

    cout << "Enter to stop." <<endl;

    // position  [rads]
    m.SetVelocity(1);

    getchar();

    m.SetVelocity(0);


}
