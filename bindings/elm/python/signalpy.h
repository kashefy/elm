/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_PYTHON_SIGNALPY_H_
#define _ELM_PYTHON_SIGNALPY_H_

#include "elm/core/signal.h"

namespace elm {

/**
 * @brief Python wrapper around Signal class
 */
class SignalPy : public Signal
{
public:
    SignalPy();
};

} // namespace elm

#endif // _ELM_PYTHON_SIGNALPY_H_
