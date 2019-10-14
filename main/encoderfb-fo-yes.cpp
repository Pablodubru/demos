#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "IPlot.h"

#include "Kinematics.h"


// Demo Closed loop with Inclination Sensor, two steps 20º incl - 0º orientation.
// It requires: -Platform inclination=0
//              -Reset IMU sensor


int main ()
{
  //--sensor--
  SerialArduino tilt;
  float incSensor,oriSensor;
//    sleep(4); //wait for sensor

  ofstream data("/home/humasoft/code/demos/graficas/encoderfb-vel-fo-i15-0o90-500g.csv",std::ofstream::out); // /home/humasoft/code/graficas

  //Samplinfg time
  double dts=0.025;
  SamplingTime Ts(dts);


//Controller
  FPDBlock con1(0.6894127,0.3171561,0.17,dts); //(kp,kd,exp,dts)
  FPDBlock con2(0.6894127,0.3171561,0.17,dts);
  FPDBlock con3(0.6894127,0.3171561,0.17,dts);

  FPDBlock reset1(con1); //Used for control reset
  FPDBlock reset2(con2);
  FPDBlock reset3(con3);
//  PIDBlock con(0,1,0,dts);



  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2048,24,0.001, 0.144);
  CiA402Device m1 (31, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
  //m1.SetupPositionMode(5,5);//set by start-pos
  m1.Setup_Velocity_Mode(5);


  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2048,24,0.001, 0.144);
  CiA402Device m2 (32, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
  //m2.SetupPositionMode(5,5);//set by start-pos
  m2.Setup_Velocity_Mode(5);

  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2048,24,0.001, 0.144);
  CiA402Device m3 (33, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
  //m3.SetupPositionMode(5,5);//set by start-pos
  m3.Setup_Velocity_Mode(5);

  double ep1,ev1,cs1;
  double ep2,ev2,cs2;
  double ep3,ev3,cs3;
  double posan1, posan2, posan3;

  double cs;

//  IPlot plot1,plot2,plot3,id;


  //--Neck Kinematics--
  double l0=0.1085;
  double lg0=l0+0.001;
  double radius=0.0075; //winch radius
  GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
  vector<double> lengths(3);

  double inc=15.0; //inclination tendon length
  double ori=0*M_PI/3; //target orientation


  //tilt initialization
  for (double t=0; t<6; t+=dts)
  {
  if (tilt.readSensor(incSensor,oriSensor)>=0)
  {
      cout << "Sensor ready" << endl<< endl;
      break;
  }


  }

  for (long stops = 5; stops > 0 ; stops--)
  {

      double interval=5.025; //in seconds
      for (double t=0;t<=interval; t+=dts)
      {
          if (tilt.readSensor(incSensor,oriSensor) <0)
          {
              cout << "Sensor error! " << endl;

          }

          neck_ik.GetIK(inc,ori,lengths);
          posan1=(lg0-lengths[0])/radius;
          posan2=(lg0-lengths[1])/radius;
          posan3=(lg0-lengths[2])/radius;
          cout << "TARGET: , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
          cout <<"orient: "<<oriSensor<<endl;
          ep1=posan1-m1.GetPosition();
          cs1=ep1 > con1;
          m1.SetVelocity(cs1);

          ep2=posan2-m2.GetPosition();
          cs2=ep2 > con2;
          m2.SetVelocity(cs2);

          ep3=posan3-m3.GetPosition();
          cs3=ep3 > con3;
          m3.SetVelocity(cs3);


//          cout << "cs1 " << cs1 << ", cs2 " << cs2 << ", cs3 " << cs3 <<endl;
          data <<t<<" , "<<inc<<" , "<<incSensor<<" , "<<ori<<" , " << oriSensor<<" , "<<cs1<<" , "<<cs2<<" , " <<cs3<<" , " <<m1.GetPosition()<<" , " <<m2.GetPosition()<<" , " <<m3.GetPosition()<< endl;

          Ts.WaitSamplingTime();
      }

      con1 = FPDBlock(reset1); //Reset?
      con2 = FPDBlock(reset2);
      con3 = FPDBlock(reset3);

      m1.SetupPositionMode(5);
      m2.SetupPositionMode(5);
      m3.SetupPositionMode(5);

      m1.SetPosition(0.01);
      m2.SetPosition(0.01);
      m3.SetPosition(0.01);

      sleep(4);
      cout<<"pos1: "<<m1.GetPosition()<<", "<<"pos2: "<<m2.GetPosition()<<", "<<"pos3: "<<m3.GetPosition()<<endl;

      m1.Setup_Velocity_Mode(5);
      m2.Setup_Velocity_Mode(5);
      m3.Setup_Velocity_Mode(5);

     ori+=22.5;
  }
  m1.SetVelocity(0);
  m2.SetVelocity(0);
  m3.SetVelocity(0);
  sleep(4);

  data.close();


return 0;

}

