#include "Block.h"

BaseBlock::BaseBlock()
{

}


long BaseBlock::SetSaturation(double low, double high)
{
    maxOut = high;
    minOut = low;

    return 0;
}



