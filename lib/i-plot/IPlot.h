#ifndef PLOT_H
#define PLOT_H


#include <vector>
#include <plotter.h>
#include <plot.h>
#include <fstream>      // std::fstream
#include <algorithm>    // std::min_element, std::max_element
#include <sstream>

#include <iostream>
//#include <plplot/plplot.h>
//#include <plplot/plstream.h>
#define NSIZE    101

class IPlot
{
public:
    IPlot();
    IPlot(double sampleTime);

    long pushBack(double new_value);

    long Plot();
    long Save(string filename);

    long Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley);
    long PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename);

    long PlotAxis(double scalex, double scaley, double offset);


    long PlotAndSave(string filename);
private:



    double Ts;

    std::vector<double> x,y;

    XPlotter plt;

    ostringstream yLabel;

    double xMax,yMax; //x and y axis maximum values

    long InitPlot();


    //PlotterParams puPlotParams;
};

#endif // PLOT_H
