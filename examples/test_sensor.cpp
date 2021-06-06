
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"
#include "imu3dmgx510.h"

int main(){
    //--sensor--
    float incSensor,oriSensor;
    double pitch,roll,sumpr;
    double dts=0.01;
    IMU3DMGX510 imu("/dev/ttyUSB0",long(1/dts));
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);


    SamplingTime tools;
    tools.SetSamplingTime(dts);



    for (double t=0;t<20;t+=dts){

        if (tilt.estimateSensor(incSensor,oriSensor)<0)
        if (imu.GetPitchRoll(pitch,roll)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            incSensor=sqrt(pitch*pitch+roll*roll);
//            oriSensor=atan2();
            cout << "incli_sen: " <<  (incSensor > filterSensor) << " , orient_sen: " << oriSensor << endl;
        }
        cout << "Available time: " << tools.WaitSamplingTime() << endl;
    }

}
