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
}

base_LayerNotSupported::base_LayerNotSupported()
    : base_Layer()
{
}

base_LayerNotSupported::base_LayerNotSupported(const LayerConfig &config, const string message)
    : base_Layer(config),
      msg_(message)
{
    ThrowException();
}

void base_LayerNotSupported::Clear()
{
    ThrowException();
}

void base_LayerNotSupported::Reset(const LayerConfig& config)
{
    ThrowException();
}

void base_LayerNotSupported::Reconfigure(const LayerConfig& config)
{
    ThrowException();
}

void base_LayerNotSupported::InputNames(const LayerIONames& io)
{
    ThrowException();
}

void base_LayerNotSupported::OutputNames(const LayerIONames& io)
{
    ThrowException();
}

void base_LayerNotSupported::Activate(const Signal &signal)
{
    ThrowException();
}

void base_LayerNotSupported::Response(Signal &signal)
{
    ThrowException();
}

void base_LayerNotSupported::ThrowException() const
{
    if(msg_.length() == 0) {
        ELM_THROW_NOT_IMPLEMENTED_WMSG(
                    string("This layer is not supported, please check the layer definition for requirements.")
                    );
    }
    else {
        ELM_THROW_NOT_IMPLEMENTED_WMSG(msg_);
    }
}
