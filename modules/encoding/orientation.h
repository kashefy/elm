#ifndef SEM_ENCODING_ORIENTATION_H_
#define SEM_ENCODING_ORIENTATION_H_

#include <core/typedefs.h>

namespace sem {

/**
 * @brief Compute single gabor kernel
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
cv::Mat1f GaborKernel(int radius,
                      float sigma,
                      float theta_rad,
                      float lambd,
                      float gamma,
                      float ps);

/**
 * @brief Compute multiple gabor kernels
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
VecMat1f GaborFilterBank(int radius,
                         float sigma,
                         const VecF &theta_rad,
                         float lambd,
                         float gamma,
                         float ps);

} // namespace sem

#endif // SEM_ENCODING_ORIENTATION_H_
