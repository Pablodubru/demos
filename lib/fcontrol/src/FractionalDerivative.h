#ifndef FRACTIONALDERIVATIVE_H
#define FRACTIONALDERIVATIVE_H

#define FRACTIONALDERIVATIVE_MAXSIZE 100

#include<valarray>
#include <iostream>      //cout
#include <math.h>       //tgamma
#include <vector>       //vector
#include "Block.h"

using namespace std;

class FractionalDerivative : public BaseBlock
{
public:
    FractionalDerivative();
    FractionalDerivative(double new_exp, double new_dts);
    double OutputUpdate(double new_input);
private:
    long Init(double new_exp, double new_dts);
    double exp;

    double dts;
    double firtol;
    valarray<double> fir;
    long N;
    std::valarray<double> convolution_n;

    //state values
    double state;
    std::valarray<double> oldStates;
    std::valarray<double> oldInputs;

    //saturation values
    double maxOut, minOut; // 0 values disables saturation


};

#endif // FRACTIONALDERIVATIVE_H
