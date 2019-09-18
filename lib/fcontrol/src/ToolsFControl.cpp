#include "ToolsFControl.h"

SamplingTime::SamplingTime(double new_dts)
{
    SetSamplingTime(new_dts);

}

//long SamplingTime::ToolsFControlInit()
//{
////    time(&actualTimeValue);
////    time(&lastTimeValue);
//    return 0;
//}

long SamplingTime::SetSamplingTime(double new_dts)
{

    dts= long(new_dts*1000*1000*1000); //dts nanoseconds
    dtsWait = chrono::nanoseconds(dts);

    return 0;
}

double SamplingTime::WaitSamplingTime()
{
    actualTimeValue = chrono::system_clock::now();

//    time(&actualTimeValue); // here time has been all way around.
//    cout <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch(lastTimeValue)).count() << endl;
    tWaited = actualTimeValue.time_since_epoch() - lastTimeValue.time_since_epoch();
//    tWaited = actualTimeValue-lastTimeValue;//chrono::nanoseconds( long(difftime(lastTimeValue,actualTimeValue)*1000*1000) );

    if(dtsWait > tWaited)
    {
        this_thread::sleep_for(dtsWait - tWaited);
    }
    else
    {
        cerr << "dtsWait Exceeded " << endl;
        cout << "dtsWait: " << dec <<dtsWait.count() << ", tWaited: " <<  tWaited.count() << endl;
        cout << "Total time: " << actualTimeValue.time_since_epoch().count() << endl;
    }


    lastTimeValue = chrono::system_clock::now();

    return (dtsWait - tWaited).count()/1000000;
}

TableInterpolation::TableInterpolation() : TableInterpolation("")
{


}

TableInterpolation::TableInterpolation(string new_table)
{


    if (new_table.size()==0)
    {
        cout << "Empty data table" << endl;
    }
    else
    {
        cout << "Data table: " << new_table << endl;
        getData(new_table);
    }
}

double TableInterpolation::GetTableValue(long row, long col)
{
    return lookupTable[row][col];

}


/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
long TableInterpolation::getData(string fileName)
{
    fstream file(fileName);
    string line = "";
    vector<double> dline;
    double value;

    lookupTable.resize(0);

    cout << "file open = " << file.is_open() << endl;
    cerr << "file open = " << file.is_open() << endl;

    // Iterate through each line and split the content using delimeter
    while (getline(file, line))
    {
//        cout << line << endl;
        istringstream ss(line);
        dline.resize(0);
//        cout << "row = " ;

//        cout << "ss: " << ss.str() << endl;
        while (ss >> value)
        {
//            cout << value <<", ";
            dline.push_back(value);
        }
        lookupTable.push_back(dline);

//        cout  << endl;
    }
    // Close the File
    file.close();

    return 0;
}


std::vector<double> PolynomialProduct(std::vector<double> p, std::vector<double> q)
{

    //long newdeg = p.size()-1 + q.size()-1; // degree of p*q

    // Special case for a polynomial of size 0
    if (p.size() == 0 or q.size() == 0)
    {
        return std::vector<double>(0);
    }

    std::vector<double> r(p.size()-1 + q.size()-1);

    for (int i = 0; i < p.size(); ++i)
    {
        for (int j = 0; j < q.size(); ++j)
        {
            r[i + j] = r[i + j] + p[i]*q[j];
        }
    }

    return r;

}
