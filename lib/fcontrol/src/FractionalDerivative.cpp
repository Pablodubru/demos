#include "FractionalDerivative.h"

FractionalDerivative::FractionalDerivative()
{

    Init(0,0.01);
}


FractionalDerivative::FractionalDerivative(double new_exp, double new_dts)
{

    //no need to use this
//    if (new_exp > 1)
//    {
//        cout << "Cant use exponents greater than one. Use another option." << endl;
//        exp=modf(new_exp,&iexp);
//        cout << "Integer exponent" << iexp << " fractional exponent: " << exp << endl;
//        Init(0,new_dts);
//        return;
//    }
    if (new_exp < 0)
    {
        cout << "Cant use exponents lesser than zero. Use another option." << endl;
//        exp=modf(new_exp,&iexp);
//        cout << "Integer exponent" << iexp << " fractional exponent: " << exp << endl;
        Init(0,new_dts);
        return;
    }
//        cout << "Fractional derivative exponent: " << new_exp << endl;

    Init(new_exp,new_dts);
    return;
}




long FractionalDerivative::Init(double new_exp, double new_dts)
{

    dts=new_dts;
    exp=new_exp;

    double bi=0;

    firtol=0.01; //default. otherwise assign with setter
    vector<double> tmp;
//    cout << "Fractional derivative FIR: "<<endl;

    for (int i=0; i<FRACTIONALDERIVATIVE_MAXSIZE; i++)
    {
        bi = tgamma(exp+1) / (tgamma(i+1)*tgamma(exp-i+1));
        tmp.push_back( pow(-1,i)*bi/(pow(dts,exp)) );
        if(abs(tmp[i])<firtol) break;
//        cout << tmp[i] << ", ";
    }
//    cout << endl<< endl;

    N=tmp.size();

    fir.resize(N,0);
    oldStates.resize(N,0);
    oldInputs.resize(N,0);

    //prepare fir for convolution (flipped values)
    for (int i=0; i<N; i++)
    {
        fir[i]=tmp[N-i-1];
//        cout << fir[i] << ", ";
    }



    return 0;
}



double FractionalDerivative::OutputUpdate(double new_input)
{
    double response=0;
//    n++;
//    if (N<n)
//    {
//        n=0;
//        Reset();

//    }

    //apply saturation
    if (maxOut!=0)
    {
        response = std::min(response,maxOut);
    }
    //apply saturation
    if (minOut!=0)
    {
        response = std::max(response,minOut);
    }

    //TODO: Check complexity for cshift: maybe deque is better for that operation

    //prepare the array for multiplication
    oldInputs = oldInputs.shift(1); //move all data one position left
    oldInputs[oldInputs.size()-1]=new_input; //overwrite last value

    //compute response
    convolution_n = (oldInputs*fir);
    response = convolution_n.sum();

//    cout  << "[ ";
//    for (int i=0; i<oldInputs.size(); i++)
//    {
//        cout << oldInputs[i] << ", " ;
//    }
//    cout  << "] "<< endl;

    state = response;
    oldStates[0]=response;//overwrite first value
    oldStates = oldStates.cshift(1); //make the first value be the last one (make it i_n)

//    cout  << "[ ";
//    for (int i=0; i<oldStates.size(); i++)
//    {
//        cout << oldStates[i] << ", " ;
//    }
//    cout  << "] "<< endl;

    return response;
}
