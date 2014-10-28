#ifndef SEM_ENCODING_GANGLION_H_
#define SEM_ENCODING_GANGLION_H_

#include <opencv2/core.hpp>

/**
 * @brief base class for modeling retinal ganglion cells
 */
class base_Ganglion
{
public:
    ~base_Ganglion();

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
    cv::Mat1f kernel_1d_;  ///< Diff. of Gaussians kernel
};

#endif // SEM_ENCODING_GANGLION_H_
