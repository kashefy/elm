/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_HIST_H_
#define _ELM_CORE_HIST_H_

#include <opencv2/core/core.hpp>

extern template class cv::Mat_<float>;

/**
 * @brief Class for wrapping OpenCV's calcHist() routine.
 * Limited to single channel matrices.
 * Useful for inspecting probability distributions
 */
class Hist1Ch
{
public:
    ~Hist1Ch();

    Hist1Ch();

    /**
     * @brief Reconfigure
     * @param histogram size (e.g. no. of bins)
     * @param range (left-closed, right-open interval)
     * @param uniformity of bins, if true, keep equally sized bins
     * @param if false, clear histograms first, otherwise accumulate from previous input
     */
    virtual void Reconfigure(int size, const std::pair<float, float> &range, bool do_uniform, bool do_accumulate);

    /**
     * @brief Compute histogram from most recent input
     * @param input
     * @param mask for excluding input elements from histogram calculation (zeros indicating elements to be ignored), default is to use all input elements.
     * @return histogram as row matrix of floats
     * @throws ExceptionTypeError for input with unsupported type
     * @throws ExceptionBadDims for multi-channel input until fully supported.
     */
    virtual cv::Mat1f Compute(const cv::Mat &in, cv::InputArray mask=cv::noArray());

protected:

    int size_;                      ///< histogram size, no. of bins
    std::pair<float, float> range_; ///< hist. range
    bool do_uniform_;               ///< uniformity of bins, if true, keep equally sized bins
    bool do_accumulate_;            ///< if false, clear histograms first, otherwise accumulate from previous input

    cv::Mat1f hist_;                  ///< computed histogram
};

#endif // _ELM_CORE_HIST_H_
