/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/io.h"

#include <algorithm>

template <class T>
void SwapEndian(T *p)
{
    unsigned char *mem = reinterpret_cast<unsigned char*>(p);
    std::reverse(mem, mem + sizeof(T));
}
