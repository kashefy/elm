/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_GANGLION_H_
#define _ELM_ENCODING_GANGLION_H_

#include <opencv2/core/core.hpp>

/**
 * @brief base class for modeling retinal ganglion cells
 */
class base_Ganglion
{
public:
    virtual ~base_Ganglion();

    /**
     * @brief Initialize ganglion cell model
     * @param spatial scale
     * @param center_on true: On center - Off surround
     */
    virtual void Init(int radius, float scale, bool center_on) = 0;

    /**
     * @brief Compute model's response to input
     * @param in
     * @return response
     */
    virtual cv::Mat1f Compute(cv::InputArray in) = 0;

protected:
    base_Ganglion();
};

/**
 * @brief Model Retinal Ganglion cells using
 * two-dimensionaldifference of gaussians class
 */
class DiffOfGaussians2dSq : public base_Ganglion
{
public:
    DiffOfGaussians2dSq();

    void Init(int radius, float scale, bool center_on);

    virtual cv::Mat1f Compute(cv::InputArray in);

    /**
     * @brief Compute Diff. of Gaussians kernel
     * @return kernel
     */
    virtual cv::Mat1f Kernel() const;

protected:
    float sigma_center_;
    float sigma_surround_;
    bool is_center_on_;
    int size_;
};

#endif // _ELM_ENCODING_GANGLION_H_
