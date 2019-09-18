#ifndef STATEVARIABLEBLOCK_H
#define STATEVARIABLEBLOCK_H

#include "Block.h"
#include "StateVariable.h"

class StateVariableBlock : public BaseBlock
{
public:
    StateVariableBlock();
    StateVariableBlock(double sampleTime);

    double OutputUpdate(double new_input);


private:


    double sT; //sample time
    StateVariable variable;

};

#endif // STATEVARIABLEBLOCK_H
