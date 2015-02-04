/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/neuron/competition.h"

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



