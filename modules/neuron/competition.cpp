#include "neuron/competition.h"

base_Competition::base_Competition()
{
}

base_Competition::~base_Competition()
{

}

base_WTA::base_WTA(float delta_t_msec)
    : base_Competition(),
      delta_t_sec_(delta_t_msec/1000.f)
{
}

base_WTA::~base_WTA()
{
}



