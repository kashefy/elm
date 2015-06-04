/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
//
// For class Exception the following applies:
//      Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
//      Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
//      Third party copyrights are property of their respective owners.
//
//      Please see details in section below.
//M*/
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include "elm/core/exception.h"

#include <opencv2/core/core.hpp>
#include <cmath>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>

using std::string;
using cv::format;
using namespace elm;

Exception::Exception()
    : code(0),
      line(0)
{}

Exception::Exception(int _code,
                     const string& _err,
                     const string& _func,
                     const string& _file,
                     int _line)
    : code(_code),
      err(_err),
      func(_func),
      file(_file),
      line(_line)
{
    formatMessage();
}

Exception::~Exception() throw() {}

const char* Exception::what() const throw() {

    return msg.c_str();
}

void Exception::formatMessage() {

    if(func.size() > 0) {
        msg = format("%s:%d: error: (%d) %s in function %s\n",
                     file.c_str(),
                     line,
                     code,
                     err.c_str(),
                     func.c_str());
    }
    else {
        msg = format("%s:%d: error: (%d) %s\n",
                     file.c_str(),
                     line,
                     code,
                     err.c_str());
    }
}

ExceptionBadDims::ExceptionBadDims(const string &msg)
    : Exception(CV_StsBadSize, msg,
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionNotImpl::ExceptionNotImpl(const string &msg)
    : Exception(CV_StsNotImplemented, msg,
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionFileIOError::ExceptionFileIOError(const string &msg)
    : Exception(CV_StsObjectNotFound, msg,
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionValueError::ExceptionValueError(const string &msg)
    : Exception(CV_StsBadArg, msg,
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionKeyError::ExceptionKeyError(const string &msg)
    : Exception(CV_StsBadArg, msg,
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionTypeError::ExceptionTypeError(const string &msg)
    : Exception(CV_StsUnsupportedFormat, msg,
                CV_Func, __FILE__, __LINE__ )
{
}
