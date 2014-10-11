#include "core/exception.h"

#include <opencv2/core/types_c.h>

using cv::String;

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
