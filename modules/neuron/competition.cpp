#include "neuron/competition.h"


base_Competition::base_Competition()
{
}

base_WTA::base_WTA(float delta_t_msec)
    : base_Competition(),
      delta_t_msec_(delta_t_msec)
{
}



