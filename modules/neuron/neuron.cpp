#include "neuron/neuron.h"

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
    else if(state == 1 || state == -1) {

        // determine spiking through poisson process
        spike = poisson_.Sample();
    }
    else {

        // TODO: unsupported state, should throw and exception
    }

    //

    return spike;
}
