#include "ts/mat_assertions.h"

#include "core/cv/mat_type_utils.h"

using namespace std;
using namespace testing;
using namespace cv;

AssertionResult IsType(const Mat &a, int mat_type)
{
    if((a.type() & CV_MAT_DEPTH_MASK) == (mat_type & CV_MAT_DEPTH_MASK)) {
        return AssertionSuccess();
    }
    else { return AssertionFailure() << MatTypeFailureMessage(a, mat_type); }
}

string MatTypeFailureMessage(const Mat& a, int mat_type) {

    stringstream failure_msg;

    failure_msg << "Mat of type " <<
                   sem::MatTypeToString(a) <<
                   " != " << sem::MatTypeToString(Mat(1, 1, mat_type));
    failure_msg << std::endl;
    return failure_msg.str();
}

AssertionResult EqualDims(const Mat &a, const Mat &b) {

    return EqualDims(a, b.size());
}

AssertionResult EqualDims(const Mat &a, const Size2i &s) {

    if(a.rows != s.height) return AssertionFailure() << "No. of rows do not match (" <<
                                                      a.rows << ") (" <<
                                                      s.height << ")";
    if(a.cols != s.width) return AssertionFailure() << "No. of columns do not match (" <<
                                                      a.cols << ") (" <<
                                                      s.width << ")";
    return AssertionSuccess();
}

string MatFailureMessageNonZero(const Mat& a, const Mat& b, const Mat& cmp) {

    stringstream failure_msg;
    for(int r=0; r<cmp.rows; r++) {

        for(int c=0; c<cmp.cols; c++) {

            if( cmp.at<uchar>(r, c) != 0 ) {

                failure_msg << "at (" << r << "," << c <<") ";

                uchar depth = a.type() & CV_MAT_DEPTH_MASK;

                switch ( depth ) {
                  case CV_8U:   failure_msg << "(" << a.at<uchar>(r, c)     << ") (" << b.at<uchar>(r, c)   << ")" ; break;
                  case CV_8S:   failure_msg << "(" << a.at<char>(r, c)      << ") (" << b.at<char>(r, c)    << ")" ; break;
                  case CV_16U:  failure_msg << "(" << a.at<uint16_t>(r, c)  << ") (" << b.at<uint16_t>(r, c) << ")" ; break;
                  case CV_16S:  failure_msg << "(" << a.at<int16_t>(r, c)   << ") (" << b.at<int16_t>(r, c) << ")" ; break;
                  case CV_32S:  failure_msg << "(" << a.at<int32_t>(r, c)   << ") (" << b.at<int32_t>(r, c) << ")" ; break;
                  case CV_32F:  failure_msg << "(" << a.at<float>(r, c)     << ") (" << b.at<float>(r, c)   << ")" ; break;
                  case CV_64F:  failure_msg << "(" << a.at<double>(r, c)    << ") (" << b.at<double>(r, c)  << ")" ; break;
                  default: break;
                }
                failure_msg << std::endl;
            }
        }
    }
    return failure_msg.str();
}

string MatFailureMessageNonZero(const Mat& a, const Mat& cmp) {

    stringstream failure_msg;
    for(int r=0; r<cmp.rows; r++) {

        for(int c=0; c<cmp.cols; c++) {

            if( cmp.at<uchar>(r, c) != 0 ) {

                failure_msg << "at (" << r << "," << c <<") ";

                uchar depth = a.type() & CV_MAT_DEPTH_MASK;

                switch ( depth ) {
                  case CV_8U:   failure_msg << "(" << a.at<uchar>(r, c)     << ")"; break;
                  case CV_8S:   failure_msg << "(" << a.at<char>(r, c)      << ")"; break;
                  case CV_16U:  failure_msg << "(" << a.at<uint16_t>(r, c)  << ")"; break;
                  case CV_16S:  failure_msg << "(" << a.at<int16_t>(r, c)   << ")"; break;
                  case CV_32S:  failure_msg << "(" << a.at<int32_t>(r, c)   << ")"; break;
                  case CV_32F:  failure_msg << "(" << a.at<float>(r, c)     << ")"; break;
                  case CV_64F:  failure_msg << "(" << a.at<double>(r, c)    << ")"; break;
                  default: break;
                }
                failure_msg << std::endl;
            }
        }
    }
    return failure_msg.str();
}

AssertionResult Equal(const Mat& a, const Mat& b) {

    AssertionResult equal_dims = EqualDims(a, b);
    if(equal_dims != AssertionSuccess()) { return equal_dims; }

    Mat cmp_out;
    compare(a, b, cmp_out, CMP_NE);
    int n = countNonZero(cmp_out);
    if(n == 0) { return AssertionSuccess(); }
    else {
        return AssertionFailure() << MatFailureMessageNonZero(a, b, cmp_out);
    }
}

AssertionResult Near(const Mat& a, const Mat& b, float tolerance) {

    AssertionResult equal_dims = EqualDims(a, b);
    if(equal_dims != AssertionSuccess()) { return equal_dims; }

    Mat cmp_out;
    Mat diff;
    absdiff(a, b, diff);
    compare(diff, tolerance, cmp_out, CMP_GT);
    int n = countNonZero(cmp_out);
    if(n == 0) { return AssertionSuccess(); }
    else { return AssertionFailure() << MatFailureMessageNonZero(a, b, cmp_out); }
}

AssertionResult LT(const Mat& a, const Mat& b) {

    AssertionResult equal_dims = EqualDims(a, b);
    if(equal_dims != AssertionSuccess()) { return equal_dims; }

    Mat cmp_out;
    compare(a, b, cmp_out, CMP_GE);
    int n = countNonZero(cmp_out);
    if(n == 0) { return AssertionSuccess(); }
    else {
        return AssertionFailure() << MatFailureMessageNonZero(a, b, cmp_out);
    }
}

AssertionResult Empty(const Mat &mat)
{
    if(mat.empty()) { return AssertionSuccess(); }
    else {
        return AssertionFailure()
                << "Mat is not empty and contains"
                << mat.total() << " elements.";
    }
}
