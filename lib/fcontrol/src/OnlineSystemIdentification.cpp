#include "OnlineSystemIdentification.h"

OnlineSystemIdentification::OnlineSystemIdentification() : OnlineSystemIdentification(1,1,0.98)
{

    //Default params: order 1, forgetting factor 0.98


}

OnlineSystemIdentification::OnlineSystemIdentification(long new_numOrder, long new_denOrder, double new_ff, double new_paramFilter)
{
    ff=new_ff;
    paramFilter = new_paramFilter;

    denOrder= new_denOrder;
    numOrder= new_numOrder;

    if(denOrder <= numOrder)
    {
        numOrder = denOrder-1;
        cout << "Only causal systems!!! Reducing numerator to order: " << numOrder << endl;
    } //now it is causal, numOrder <= denOrder

    //share orders available, allowing den use numerator unused orders.
    if( (denOrder + numOrder ) > rlms_N)
    {
        if (numOrder < rlms_N/2) //Give num orders to den.
        {
            denOrder = rlms_N - numOrder;
            cout << "Maximum order reached. Reducing orders to: num = " << numOrder << ", den =" << denOrder << endl;
        }
        else //The only possibility is to reduce both to maximum size.
        {
            denOrder = rlms_N/2;
            numOrder = denOrder-1;
            cout << "Maximum order reached. Reducing orders to: num = " << numOrder << ", den = " << denOrder << endl;


        }
    }

    order = numOrder+1 + denOrder; //+1 include numerator actual input
    phiNumIndex = denOrder+1-1;//num next to den +1 and starts at zero -1.
    phiLastIndex = order-1; //starts at zero -1.

//    //    P.resize(order,NoChange); //no longer needed
//    P=P.Random(order,order);
//    cout << "--> Initial P <--" << endl << P << endl;
//    //    phi.resize(order,order); //no longer needed
//    phi=phi.Random(order,1);
//    cout << "--> Initial phi <--" << endl << phi << endl;
//    //    L.resize(order,NoChange);//no longer needed
//    L=L.setZero(order,1);
//    cout << "--> Initial L <--" << endl << L  << endl;

//    th=th.setZero(order,1);
//    cout << "--> Initial th <--" << endl << th << endl << "--------------order : " << numOrder << " / " << denOrder << endl;

    //    P.resize(order,NoChange); //no longer needed
    R=R.Random(order,order);
    //    phi.resize(order,order); //no longer needed
    phi=100*phi.Random(order,1);
    R_ev=100*R_ev.Random(order,1);
    //    L.resize(order,NoChange);//no longer needed
    L=L.setZero(order,1);

    th=th.setZero(order,1);


//    cout << "--> Initial R <--" << endl << R << endl;
//    cout << "--> Initial phi <--" << endl << phi << endl;
    cout << "--> Initial th <--" << endl << th << endl << "--------------order : " << numOrder << " / " << denOrder << endl;
//    cout << "--> Initial L <--" << endl << L  << endl;

    idNum.resize(numOrder+1);
    idDen.resize(denOrder+1);

    filterOn=0;



}

OnlineSystemIdentification::OnlineSystemIdentification(long new_numOrder, long new_denOrder,SystemBlock new_filter)
: OnlineSystemIdentification(new_numOrder,new_denOrder,0.98)
{

    SetFilter(new_filter);
    //Default params: order 1, forgetting factor 0.98


}

OnlineSystemIdentification::OnlineSystemIdentification(long new_numOrder, long new_denOrder,SystemBlock new_filter, double new_ff, double new_paramFilter)
: OnlineSystemIdentification(new_numOrder,new_denOrder, new_ff, new_paramFilter)
{

    SetFilter(new_filter);
    //Default params: order 1, forgetting factor 0.98


}


long OnlineSystemIdentification::SetFilter(SystemBlock filter)
{
    inFilter = SystemBlock(filter);
    outFilter = SystemBlock(filter);
    filterOn=1;
    return 0;
}


//double OnlineSystemIdentification::UpdateSystem(double input, double output)
//{

//    ti++;
//    //move all phi input data one position backwards to have inputs from actual to last needed past values.
//    for (int i=numOrder-1; i>0; i--) //Leave the first place empty for current input value
//    {
//        //input indexes can start from zero to include actual input??
//        phi[i+denOrder] = phi[i+denOrder-1];
//    }
//    phi(0+denOrder)=input;

//    //        cout << "b0 " << (out[ti]-phi.transpose()*th)/in[ti] << ", at step: " <<  ti << endl;
//    //        cout << "phi: " << phi.transpose() << endl;

//    L = (P*phi)/(ff+(phi.transpose()*P*phi));
//    //    cout << "L " << L.transpose() << ", at step: " <<  ti << endl;

//    P=(1/ff)*(P-L*phi.transpose()*P);
//    //    cout << "P " << P << ", at step: " <<  ti << endl;

//    err=output-phi.transpose()*th;
//    //    cout << "Error preditcion " << err << ", at step: " <<  ti << endl;

//    th=th+(L*err);
//    //    cout << "Params " << th.transpose() << endl;

//    //move all phi output data one position backwards for the next iteration
//    for (int i=denOrder-1; i>0; i--)
//    {
//        phi[i] = phi[i-1];
//    }
//    //and add the actual value
//    phi(0)=output;
//    return err;

//}


//Version not considering actual input
double OnlineSystemIdentification::UpdateSystemDT1(double input, double output)
{
    //!!!Not working, needs revision!!!

    cout <<    "!!!Not working, needs revision!!!" << endl;

    //Assuming that input refers to u_{t-1} and output refers to y_{t-1}
    ti++;
    //move all phi input data one position backwards to have inputs from actual to last needed past values.
    for (int i=phiLastIndex; i>0; i--)
    {
//        cout << "phiLastIndex: " << phiLastIndex << " ; phiNumIndex: " << phiNumIndex << endl;
//        cout << "i: " << i << " ; phi[i]: " << phi[i] << endl;

        phi[i] = phi[i-1];
    }
    phi(phiNumIndex)=input;
    phi(0)=-output;
//            cout << "b0 " << (output[ti]-phi.transpose()*th)/input[ti] << ", at step: " <<  ti << endl;
    cout << "phi: " << phi.transpose() << endl;



    R = ff*R + phi*phi.transpose();
//    cout << "R: " << endl << R << endl;
//    cout << "R^-1: " << endl << R.inverse() << endl;
//    cout << "|R|: " << R.determinant() << endl;


    R_ev=((phi*phi.transpose()).eigenvalues()).real();


//    if (!(phiEigenvalues.prod()>0 && phiEigenvalues.sum()>1))
//    if (phiEigenvalues.minCoeff()<-1E-15 || phiEigenvalues.maxCoeff()<0.1)
    if (R_ev.minCoeff()<=0 || R_ev.maxCoeff()<0.1)
    {
//        cout << "phi (" << phi.transpose() <<  endl;
        cout << "PHI BAD POSED (" << R_ev.transpose() << ") at: " << ti << endl;
        phi(0)=-output;
        return 0;
    }






    th = th + R.inverse()*phi*(output - phi.transpose()*th);
//    cout << "th: " << th.transpose() << endl;
//    cout << "phi: " << phi.transpose() << endl;
//    cout << "test: phiT*theta " << phi.transpose()*th << endl;


    return err;

}


//Version considering actual output y_{t} previous input u_{t-1}
double OnlineSystemIdentification::UpdateSystem(double new_input, double new_output)
{

    //Assuming that input refers to u_{t-1} and output refers to y_{t}

    if (filterOn)
    {
        input=new_input > inFilter;
        output=new_output > outFilter;
//            cout << "phi(phiNumIndex): " << phi(phiNumIndex) << endl;

    }
    else
    {
        input=new_input;
        output=new_output;
//            cout << "phi(phiNumIndex): " << phi(phiNumIndex) << endl;

    }

    ti++;

    //Update phi input data
    for (int i=phiLastIndex; i>phiNumIndex; i--) //order-1 is full phi size
    {
        //input indexes can start from zero to include actual input??
       // cout << "phiLastIndex: " << phiLastIndex << " ; phiNumIndex: " << phiNumIndex << endl;
        phi[i] = phi[i-1];
    }

    phi(phiNumIndex)=input;


//            cout << "b0 " << (output[ti]-phi.transpose()*th)/input[ti] << ", at step: " <<  ti << endl;
//    cout << "phi: " << phi.transpose() << endl;




    newR = phi*phi.transpose();



//    cout << "R: " << endl << R << endl;
//    cout << "R^-1: " << endl << R.inverse() << endl;
//    cout << "|R|: " << R.determinant() << endl;

//    if (abs(R.determinant()) < 100)
//    {
//        cout << "|R|: " << R.determinant() << endl;
//        return 0;
//    }



    R_ev=(newR.eigenvalues()).real();

//    if (!(phiEigenvalues.prod()>0 && phiEigenvalues.sum()>1))
//    if (phiEigenvalues.minCoeff()<-1E-15 || phiEigenvalues.maxCoeff()<0.1)
    if (R_ev.minCoeff()<=0 || R_ev.maxCoeff()<0.1)
    {
//        cout << "phi (" << phi.transpose() <<  endl;
//        cout << "PHI BAD POSED (" << phiEigenvalues.transpose() << ") at: " << ti << endl;
        phi(0)=-output;
        return -1;
    }

    //updating R after check keeps it unchanged for non persistent exciting inputs
    R = ff*R + newR;


    th = th + paramFilter*R.inverse()*phi*(output - phi.transpose()*th);
//    cout << "th: " << th.transpose() << endl;

//    cout << "phi: " << phi.transpose() << endl;
//    cout << "test: phiT*theta " << phi.transpose()*th << endl;

    //Update phi output data for next iteration
    for (int i=denOrder-1; i>0; i--)
    {
        phi[i] = phi[i-1];
    }

    phi(0)=-output;

    //and add the actual value


    return err;

}


long OnlineSystemIdentification::GetSystemBlock(SystemBlock & idsys)
{
    vector<double> num(numOrder+1),den(denOrder+1);
    GetZTransferFunction(num,den);
    idsys = SystemBlock(num,den);
    return 0;
}


double OnlineSystemIdentification::GetZTransferFunction(vector<double> &num, vector<double> &den)
{

//    for (int i=0; i<numOrder; i++)
//    {
//        num[i]=th[i];
//        //        cout << "num i= " << i << " num: " << num[i] << " th: " << th(i)<<endl;
//    }

//    for (int i=0; i<denOrder; i++)
//    {
//        den[i]=th[i+numOrder];
//        //        cout << "den i= " << i << " den: " << den[i] << " th: " << th(i+numOrder)<<endl;
//    }
//    return err;
//    cout << "th: " << th.transpose() << endl;

    num[0]=th[phiLastIndex];
//    cout << "fcontrol num=[ " ;//<< num[0] ;
    for (int i=0; i<=numOrder; i++)
    {
        num[i]=th[phiLastIndex-i];
//        cout << num[i]<< ", " ;
    }
//    cout  << "], den=[ ";


    for (int i=0; i<denOrder; i++)
    {
        den[i]=th[phiNumIndex-1-i];
//        cout << den[i]<< ", " ;

    }
    den[denOrder]=1;
//    cout << den[denOrder] << "] "<< endl;

    return err;

}


double OnlineSystemIdentification::PrintZTransferFunction(double dts)
{

    //todo: scilab format
//    cout << "G=tf([ " << th[0] ;
//    for (int i=1; i<numOrder; i++)
//    {
//        cout << ", " << th[i];
//    }
//    cout << "],[ "<< th[numOrder];
//    for (int i=1; i<denOrder; i++)
//    {
//        cout << ", " << th[i+numOrder];

//    }
//    cout << "]," <<dts<< ")"<< endl;

//    cout << "th: " << th.transpose() << endl;

//    cout << "matlab G=tf([ " << th[phiNumIndex] ;
//    for (int i=phiNumIndex+1; i<=phiLastIndex; i++)
//    {
//        cout << ", " << th[i];
//    }
//    cout << "],[ " << 1;
//    for (int i=0; i<phiNumIndex; i++)
//    {
//        cout << ", " << th[i];

//    }
//    cout << "]," <<dts<< ")"<< endl;


    GetZTransferFunction(idNum,idDen);

    cout << "matlab G=tf([ " << idNum.back() ;
    for (int i=idNum.size()-2; i>=0; i--)
    {
        cout << ", " << idNum[i];
    }
    cout << "],[ " << idDen.back();
    for (int i=idDen.size()-2; i>=0; i--)
    {
        cout << ", " << idDen[i];

    }
    cout << "]," <<dts<< ")"<< endl;

    return err;



}


double OnlineSystemIdentification::PrintParamsVector()
{
    cout << "Parameters vector: [";
    for (int i=0; i<order; i++)
    {
        cout << th[i] << ", ";
    }
    cout << "]" << endl;
    return err;
}

long OnlineSystemIdentification::GetMagnitudeAndPhase(double dts, double w, double &magnitude, double &phase)
{
    z=exp(dts*w*complex<double>(0,1));
    double numi=0, deni=0;
//    cout << "fcontrol num=[ " ;//<< num[0] ;
    for (int i=0; i<=numOrder; i++)
    {
//        num[i]=th[phiLastIndex-i];
//        cout << num[i]<< ", " ;
        numi=th[phiLastIndex-i];
        nz=nz+numi*pow(z,i);


    }
//    cout  << "], den=[ ";


    for (int i=0; i<denOrder; i++)
    {
//        den[i]=th[phiNumIndex-1-i];
//        cout << den[i]<< ", " ;
        deni=th[phiNumIndex-1-i];
        dz=dz+deni*pow(z,i);

    }
//    den[denOrder]=1;
    dz=dz+pow(z,denOrder);

    //phase and magnitude
    magnitude = abs(nz/dz);
    phase = arg(nz/dz);

    return 0;
}

vector<double> OnlineSystemIdentification::GetParamsVector()
{
    vector<double> params(order);
    for (int i=0; i<order; i++)
    {
        params[i]=th(i);
    }
    return params;
}
