#ifndef SEM_CORE_EXCEPTION_H_
#define SEM_CORE_EXCEPTION_H_

#include <string>
#include <opencv2/core.hpp>

#define SEM_THROW_BAD_DIMS(msg) throw ExceptionBadDims(msg)
#define SEM_THROW_NOT_IMPLEMENTED throw ExceptionNotImpl("Not implemented yet.")

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

#endif // SEM_CORE_EXCEPTION_H_
