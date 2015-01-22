/** @file Run Hierarchical SEM simulation (Kasehf, Masters Thesis 2013)
 */
#include <iostream>

#include "sem/core/core.h"
#include "simulationhsem.h"

using namespace std;

int main() {

    cout<<sem::GetVersion()<<endl;

    SimulationHSEM s;

    cout<<"Learn()"<<endl;

    s.Learn();

    cout<<"Test()"<<endl;

    //s.Test();

    cout<<"Eval()"<<endl;

    s.Eval();

    return 0;
}
