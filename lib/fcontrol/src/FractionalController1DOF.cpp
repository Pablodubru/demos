#include "FractionalController1DOF.h"

FractionalController1DOF::FractionalController1DOF()
{

}

FractionalController1DOF::FractionalController1DOF(double new_exp, double new_dts)
{
    dts=new_dts;
    if (new_exp < 0)
    {
//        cout << "Using fractional derivative for exponents lesser than one. Splitting." << endl;
        exp=1+modf(new_exp,&iexp);
        iexp--; //add one integrator
//        cout << "Integer exponent: " << iexp << " fractional exponent: " << exp << endl;
        //initialize the integer part
        ipart = SystemBlock(
//                    std::vector<double> {dts/2,dts/2},
//                    std::vector<double> {-1,1});
                    std::vector<double> {0,dts*1},
                    std::vector<double> {-1,1});

        Init(exp,new_dts);
        return;
    }
    if (new_exp > 1)
    {
//        cout << "Using fractional derivative for exponents greater than one. Splitting." << endl;
//        exp=modf(new_exp,&iexp);
//        cout << "Integer exponent: " << iexp << " fractional exponent: " << exp << endl;
//        //initialize the integer part

//        // LPF implementation
//        double N = 20;    // LPFfilter N
//        ipart = SystemBlock(
//                    std::vector<double> {-1*N,1*N},
//                    std::vector<double> {-1,1+N*dts*1});
//        //                std::vector<double> {-1,  1},
//        //                std::vector<double> {0,      Ts*1});
        exp =new_exp;
        Init(exp,new_dts);
        return;
    }

    //otherwise
    exp = new_exp;
//    cout << "Integer exponent: " << 0 << " fractional exponent: " << exp << endl;
    ipart = SystemBlock(
                std::vector<double> {1},
                std::vector<double> {1});
    Init(exp,new_dts);
    return;


}

double FractionalController1DOF::OutputUpdate(double new_input)
{
    state = new_input > ipart > fpart;
//    cout << "new_input " << new_input << " state " << state << endl;

    return state;

}


long FractionalController1DOF::Init(double new_exp, double new_dts)
{

    dts=new_dts;
    exp=new_exp;
    fpart = FractionalDerivative (exp, dts);






    return 0;
}

double FractionalController1DOF::getState() const
{
    return state;
}


