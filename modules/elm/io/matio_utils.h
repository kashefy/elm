/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_MATIO_UTILS_H_
#define _ELM_IO_MATIO_UTILS_H_

#ifdef __WITH_MATIO

#include "matio.h"

namespace elm {

unsigned int MATIOClassTOCV_TYPE(matio_classes type);

} // namespace elm

#endif // __WITH_MATIO

#endif // _ELM_IO_MATIO_UTILS_H_
