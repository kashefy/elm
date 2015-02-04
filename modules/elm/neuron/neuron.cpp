/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/neuron/neuron.h"

#include "elm/core/exception.h"

YNeuron::YNeuron()
    : poisson_(0.f, 0.f) // dummy initialization, ugly
{
}

void YNeuron::init(float frequency, float delta_t_msec)
{
    poisson_ = PoissonProcess(frequency, delta_t_msec);
}

int YNeuron::State(float state)
{
    int spike;

    if(state == 0) {

        spike = 0; // don't spike
    }
    else if(state == 1.f || state == -1.f) {

        // determine spiking through poisson process
        spike = poisson_.Sample();
    }
    else {
        ELM_THROW_NOT_IMPLEMENTED_WMSG("Analog state not supported.");
    }

    return spike;
}
