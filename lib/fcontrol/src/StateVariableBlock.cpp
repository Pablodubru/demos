#include "StateVariableBlock.h"


StateVariableBlock::StateVariableBlock()
{

    variable = StateVariable(0,0,0);
    sT=0.01; //make sure sT has a value.
}

StateVariableBlock::StateVariableBlock(double sampleTime)
{
    StateVariableBlock();
    sT=sampleTime;

}

double StateVariableBlock::OutputUpdate(double new_input)
{
    variable.Update(new_input, sT);
    return variable.D(0);

}
