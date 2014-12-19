#ifndef SEM_ENCODING_ORIENTATION_H_
#define SEM_ENCODING_ORIENTATION_H_

#include "core/typedefs.h"
#include "encoding/base_filterbank.h"

class GaborFilterBank : public base_FilterBank
{
public:
    GaborFilterBank();

    int Reset(int radius,
              float sigma,
              const VecF &theta_rad,
              float lambd,
              float gamma,
              float ps);

    /**
     * @brief Create a single gabor kernel
     * Appropriate model for orientation selective V1 simple cells and edge detectors
     *
     * @param kernel radius, yields kernel of size (r*2+1, r*2+1)
     * @param sigma Std dev. of gaussian envelope
     * @param theta_rad filter's preferred orientation in radians (0 rad = horizontal, pi/2 = vertical)
     * @param lambd wavelength of sinusoid
     * @param gamma spatial aspect ratio
     * @param ps phase offset in radians
     * @return kernel in 32-bit floats
     */
    static cv::Mat1f CreateKernel(int radius,
                                  float sigma,
                                  float theta_rad,
                                  float lambd,
                                  float gamma,
                                  float ps);

    /**
     * @brief Create multiple gabor kernels
     * Appropriate for constructing a filter bank to use in an orientation map
     *
     * @param kernel radius, yields kernel of size (r*2+1, r*2+1)
     * @param sigma Std dev. of gaussian envelope
     * @param theta_rad vector of preferred orientation in radians (0 rad = horizontal, pi/2 = vertical)
     * @param lambd wavelength of sinusoid
     * @param gamma spatial aspect ratio
     * @param ps phase offset in radians
     * @return kernel in 32-bit floats
     */
    static VecMat1f CreateKernels(int radius,
                                  float sigma,
                                  const VecF &theta_rad,
                                  float lambd,
                                  float gamma,
                                  float ps);

    VecMat1f Kernels() const;

protected:
    /**
     * @brief Rectify response to individual kernels by squaring the response
     * @param response squared in place
     */
    void Rectify(cv::Mat1f &response);

}; // GaborFilterBank

#endif // SEM_ENCODING_ORIENTATION_H_
