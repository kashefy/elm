#ifndef SEM_CORE_EXCEPTION_H_
#define SEM_CORE_EXCEPTION_H_

#include <string>
#include <opencv2/core.hpp>

/** Preprocessor macros for throwing types of Exceptions with custom error messages
 */
#define SEM_THROW_BAD_DIMS(msg) throw sem::ExceptionBadDims(msg)
#define SEM_THROW_NOT_IMPLEMENTED throw sem::ExceptionNotImpl("Not implemented yet.")
#define SEM_THROW_NOT_IMPLEMENTED_WMSG(msg) throw sem::ExceptionNotImpl(msg)
#define SEM_THROW_FILEIO_ERROR(msg) throw sem::ExceptionFileIOError(msg)
#define SEM_THROW_VALUE_ERROR(msg) throw sem::ExceptionValueError(msg)
#define SEM_THROW_KEY_ERROR(msg) throw sem::ExceptionKeyError(msg)
#define SEM_THROW_TYPE_ERROR(msg) throw sem::ExceptionTypeError(msg)

namespace sem {

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

} // namespace sem

#endif // SEM_CORE_EXCEPTION_H_
