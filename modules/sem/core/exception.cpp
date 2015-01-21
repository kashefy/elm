#include "sem/core/exception.h"

#include <opencv2/core/types_c.h>

using cv::String;
using namespace sem;

Exception::Exception(int _code, const std::string &msg,
                     const std::string &_func, const std::string &_file, int _line)
    : cv::Exception(_code, String(msg),
                    String(_func), String(_file), _line)
{
}

ExceptionBadDims::ExceptionBadDims(const std::string &msg)
    : Exception(CV_StsBadSize, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionNotImpl::ExceptionNotImpl(const std::string &msg)
    : Exception(CV_StsNotImplemented, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionFileIOError::ExceptionFileIOError(const std::string &msg)
    : Exception(CV_StsObjectNotFound, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionValueError::ExceptionValueError(const std::string &msg)
    : Exception(CV_StsBadArg, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionKeyError::ExceptionKeyError(const std::string &msg)
    : Exception(CV_StsBadArg, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}

ExceptionTypeError::ExceptionTypeError(const std::string &msg)
    : Exception(CV_StsUnsupportedFormat, String(msg),
                CV_Func, __FILE__, __LINE__ )
{
}
