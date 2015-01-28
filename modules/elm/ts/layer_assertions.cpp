/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace elm;

void elm::ValidateRequiredIONames(const MapIONamesB &io_pairs, shared_ptr<base_Layer> &layer_ptr)
{
    VecS keys = Keys(io_pairs);

    for(VecS::iterator itr1=keys.end(); itr1 != keys.begin(); --itr1) {

        VecS subset(keys.begin(), itr1);
        do {

            // populate layer io names with subset
            LayerIONames io;
            for(size_t i=0; i<subset.size(); i++) {

                IOName _v = io_pairs.at(subset[i]);

                if(!_v.first) {
                    io.Input(subset[i], _v.second);
                }
                else {
                    io.Output(subset[i], _v.second);
                }
            }

            if(subset.size() == io_pairs.size()) {

                EXPECT_NO_THROW(layer_ptr->IONames(io));
            }
            else {

                EXPECT_THROW(layer_ptr->IONames(io), ExceptionKeyError);
            }

        } while ( next_permutation(subset.begin(), subset.end()) );
    }
}


