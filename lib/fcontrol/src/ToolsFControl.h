#ifndef TOOLSFCONTROL_H
#define TOOLSFCONTROL_H

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <fstream>      // std::ifstream
#include <sstream>      // std::istringstream

using namespace std;
class SamplingTime
{
public:
    SamplingTime(double new_dts = 0.01);
//    long ToolsFControlInit();
    long SetSamplingTime(double new_dts);
    double WaitSamplingTime();

private:
    chrono::system_clock::time_point actualTimeValue, lastTimeValue; //last time value
//    long timeAlreadyWaited; //in nanoseconds
    long dts; //sampling time nanoseconds.
    chrono::nanoseconds dtsWait, tWaited;
//    chrono::system_clock clock;

};

class TableInterpolation
{
public:
    TableInterpolation();
    TableInterpolation(string new_table);
    double GetTableValue(long row, long col);
private:
    long getData(string fileName);
    vector < vector<double> > lookupTable;

};

#endif // TOOLSFCONTROL_H
