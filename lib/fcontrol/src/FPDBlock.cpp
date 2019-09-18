#include "FPDBlock.h"

FPDBlock::FPDBlock()
{
    FPDBlock(1, 1, 1, 0.001);
}

FPDBlock::FPDBlock(double new_kp, double new_kd, double new_fex, double new_dts)
{
    kp=new_kp;
    kd=new_kd;
    fex=new_fex;
    dts= new_dts;
    s_e = FractionalController1DOF(fex,dts);

}

double FPDBlock::OutputUpdate(double new_input)
{
//    double cp,ci,cd;
    input=new_input;

    state=input*kp + kd*s_e.OutputUpdate(input);

//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    if (!isnormal(state)) return 0;
    return state;

}

double FPDBlock::getState() const
{
    return state;
}
