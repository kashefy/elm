/** @file Run SEM simulation (NIPS 2010)
 */
#include <iostream>

#include "sem/core/core.h"
#include "sem/simulationsem.h"

using namespace std;

int main() {

    cout<<sem::GetVersion()<<endl;

    SimulationSEM s;

    cout<<"Learn()"<<endl;

    s.Learn();

    cout<<"Test()"<<endl;

    //s.Test();

    cout<<"Eval()"<<endl;

    s.Eval();

    return 0;
}
