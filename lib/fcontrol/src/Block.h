#ifndef BLOCK_H
#define BLOCK_H


#include <vector>
#include <deque>
#include <valarray>
#include <math.h>
#include <iostream>

class BaseBlock
{
public:
    BaseBlock();

    friend double operator > (double input, BaseBlock& output)
    {
        return output.OutputUpdate(input);
    }

    long SetSaturation(double low, double high);

protected:
    double maxOut, minOut; // 0 values disables saturation


private:

    double saturation;
    //saturation values

    virtual double OutputUpdate(double new_input)=0;




};

#endif // BLOCK_H
