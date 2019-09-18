#include "PIDBlock.h"

using  namespace std;
long PIDBlock::Initial(double new_Ts)
{

    Ts=new_Ts;
        std::cout << "Ts : " <<Ts << std::endl;
    iBlock = SystemBlock(
                std::vector<double> {Ts/2,Ts/2},
                std::vector<double> {-1,1});
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1});

        // LPF implementation
    double N = 20;    // LPFfilter N
    dBlock = SystemBlock(
                std::vector<double> {-1*N,1*N},
                std::vector<double> {-1,1+N*Ts*1});
//                std::vector<double> {-1,  1},
//                std::vector<double> {0,      Ts*1});
    return 0;
}

PIDBlock::PIDBlock()
{

    Initial(0.01);

    kp=1;
    ki=1;
    kd=1;

}

PIDBlock::PIDBlock(double new_kp, double new_ki, double new_kd, double new_Ts)
{
    Ts=new_Ts;
    Initial(new_Ts);

    kp=new_kp;
    ki=new_ki;
    kd=new_kd;


}

double PIDBlock::UpdateControl(double input)
{
    double cp,ci,cd;

    cp=input*kp;
    ci = ki*iBlock.OutputUpdate(input);
    cd = kd*dBlock.OutputUpdate(input);

    if (signbit(ci)!=signbit(input))
    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
    }

    state=cp+ci+cd;

//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    return state;

}

double PIDBlock::OutputUpdate(double input)
{

    return UpdateControl(input);
}

double PIDBlock::GetState() const
{
    return state;
}

//void PIDBlock::operator>>(double &output)
//{
//    output = state;

//}

//void PIDBlock::operator>>(SystemBlock &output)
//{
//    output.OutputUpdate(state);

//}
