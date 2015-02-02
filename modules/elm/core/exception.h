/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_EXCEPTION_H_
#define _ELM_CORE_EXCEPTION_H_

#include <string>
#include <opencv2/core.hpp>

/** Preprocessor macros for throwing types of Exceptions with custom error messages
 */
#define ELM_THROW_BAD_DIMS(msg) throw elm::ExceptionBadDims(msg)
#define ELM_THROW_NOT_IMPLEMENTED throw elm::ExceptionNotImpl("Not implemented yet.")
#define ELM_THROW_NOT_IMPLEMENTED_WMSG(msg) throw elm::ExceptionNotImpl(msg)
#define ELM_THROW_FILEIO_ERROR(msg) throw elm::ExceptionFileIOError(msg)
#define ELM_THROW_VALUE_ERROR(msg) throw elm::ExceptionValueError(msg)
#define ELM_THROW_KEY_ERROR(msg) throw elm::ExceptionKeyError(msg)
#define ELM_THROW_TYPE_ERROR(msg) throw elm::ExceptionTypeError(msg)

namespace elm {

class Exception : public cv::Exception
{
public:
    Exception(int _code, const std::string &msg,
              const std::string &_func, const std::string &_file, int _line);
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
