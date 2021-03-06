/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_PYTHON_SIGNALPY_H_
#define _ELM_PYTHON_SIGNALPY_H_

#include <boost/python/dict.hpp>
#include <boost/python/numeric.hpp>

#include "elm/core/signal.h"

namespace elm {

/**
 * @brief Python wrapper around Signal class
 */
class SignalPy : public Signal
{
public:
    SignalPy();

    /**
     * @brief convert Signal content to Python dict
     * @return signal content
     */
    boost::python::dict toPythonDict() const;

    /**
     * @brief add dict to signal content
     * @param source dict
     */
    void fromPythonDict(const boost::python::dict &d);
};

} // namespace elm

#endif // _ELM_PYTHON_SIGNALPY_H_
