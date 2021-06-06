
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"
#include "imu3dmgx510.h"

int main(){
    //--sensor--
    double incSensor,oriSensor;
    double pitch,roll,sumpr;
    double dts=0.02;
    IMU3DMGX510 imu("/dev/ttyUSB0",long(1/dts));
//    SystemBlock filterSensor(0.09516,0,- 0.9048,1);


    SamplingTime tools;
    tools.SetSamplingTime(dts);



    /*for (double t=0;t<100;t+=dts){

        if (imu.GetPitchRoll(pitch,roll)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            incSensor=(180/M_PI)*sqrt(pitch*pitch+roll*roll);
            oriSensor = (180/M_PI)*(atan2(roll,pitch)+M_PI);
//            oriSensor=atan2();
            cout << "incli_sen: " <<  (incSensor ) << " , orient_sen: " << oriSensor << endl;
        }
        tools.WaitSamplingTime();
    }*/

    for (double t=0;t<20;t+=dts){

        if (imu.GetIncliOri(incSensor,oriSensor)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            cout << "incli_sen: " <<  (incSensor ) << " , orient_sen: " << oriSensor << endl;
        }
        tools.WaitSamplingTime();
    }

}
