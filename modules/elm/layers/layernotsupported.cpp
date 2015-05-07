/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layernotsupported.h"

#include "elm/core/exception.h"

using std::string;

using namespace elm;

base_LayerNotSupported::base_LayerNotSupported(const string &message)
    : base_Layer(),
      msg_(message)
{
    ThrowException();
}//LCOV_EXCL_LINE

base_LayerNotSupported::base_LayerNotSupported()
    : base_Layer()
{
}

void base_LayerNotSupported::Clear()
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::Reset(const LayerConfig& config)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::Reconfigure(const LayerConfig& config)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::InputNames(const LayerInputNames& io)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::OutputNames(const LayerOutputNames& io)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::Activate(const Signal &signal)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::Response(Signal &signal)
{
    ThrowException();
}//LCOV_EXCL_LINE

void base_LayerNotSupported::ThrowException() const
{
    if(msg_.length() == 0) {
        ELM_THROW_NOT_IMPLEMENTED_WMSG(
                    string("This layer is not supported, please check the layer definition for requirements.")
                    );
    }//LCOV_EXCL_LINE
    else {
        ELM_THROW_NOT_IMPLEMENTED_WMSG(msg_);
    }//LCOV_EXCL_LINE
}//LCOV_EXCL_LINE
