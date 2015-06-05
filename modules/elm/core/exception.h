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
#ifndef _ELM_CORE_EXCEPTION_H_
#define _ELM_CORE_EXCEPTION_H_

#include <string>
#include <exception>

/** Preprocessor macros for throwing types of Exceptions with custom error messages
 */
#define ELM_THROW_BAD_DIMS(msg) throw elm::ExceptionBadDims(msg)
#define ELM_THROW_NOT_IMPLEMENTED throw elm::ExceptionNotImpl("Not implemented yet.")
#define ELM_THROW_NOT_IMPLEMENTED_WMSG(msg) throw elm::ExceptionNotImpl(msg)
#define ELM_THROW_FILEIO_ERROR(msg) throw elm::ExceptionFileIOError(msg)
#define ELM_THROW_VALUE_ERROR(msg) throw elm::ExceptionValueError(msg)
#define ELM_THROW_KEY_ERROR(msg) throw elm::ExceptionKeyError(msg)
#define ELM_THROW_TYPE_ERROR(msg) throw elm::ExceptionTypeError(msg)

/** Preprocessor macros for throwing types of Exceptions with custom error messages
 *  on condition evaluating to true
 */
#define ELM_THROW_BAD_DIMS_IF(condition, msg) if(condition) ELM_THROW_BAD_DIMS(msg)

namespace elm {

/**
 * @brief The Exception class adapted from OpenCV's Exception class
 */
class Exception : public std::exception
{
public:
    /**
     * @brief Default constructor
     */
    Exception();

    /**
     * @brief Full constructor. Normally the constuctor is not called explicitly.
     * Instead, the macros ELM_THROW_* are used.
     * @param _code error code
     * @param _err error message
     * @param _func function in which the error occured
     * @param _file file in which the function is located
     * @param _line the line in the file where the error was detected
     */
    Exception(int _code,
              const std::string& _err,
              const std::string& _func,
              const std::string& _file,
              int _line);

    virtual ~Exception() throw();

    /**
     * @brief the error description and the context as a text string
     * @return message describing the error
     */
    virtual const char *what() const throw();

    /** @brief format the error message
     */
    void formatMessage();

    std::string msg;    ///< the formatted error message

    int code;           ///< error code @see CVStatus
    std::string err;         ///< error description
    std::string func;        ///< function name. Available only when the compiler supports getting it
    std::string file;        ///< source file name where the error has occured
    int line;           ///< line number in the source file where the error has occured
};

class ExceptionBadDims : public Exception
{
public:
    ExceptionBadDims(const std::string &msg);
};

class ExceptionNotImpl : public Exception
{
public:
    ExceptionNotImpl(const std::string &msg);
};

class ExceptionFileIOError : public Exception
{
public:
    ExceptionFileIOError(const std::string &msg);
};

class ExceptionValueError : public Exception
{
public:
    ExceptionValueError(const std::string &msg);
};

class ExceptionKeyError : public Exception
{
public:
    ExceptionKeyError(const std::string &msg);
};

class ExceptionTypeError : public Exception
{
public:
    ExceptionTypeError(const std::string &msg);
};

} // namespace elm

#endif // _ELM_CORE_EXCEPTION_H_
