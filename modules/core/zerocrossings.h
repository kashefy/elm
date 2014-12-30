#ifndef SEM_CORE_ZEROCROSSINGS_H_
#define SEM_CORE_ZEROCROSSINGS_H_

#include <opencv2/core.hpp>

/**
 * @brief base class for computing zero corssings
 */
class base_ZeroCrossings
{
public:
    ~base_ZeroCrossings();

    virtual void operator ()(const cv::Mat1f &src, cv::Mat1f &dst) const = 0;

protected:
    base_ZeroCrossings();
};

/**
 * @brief Use forward difference (1st derivative) to detect zero crossings
 * src[i]*src[i+1] < 0 indicates a zero crossing.
 * Perform on both x and y axes.
 */
class ZeroCrossingsDiff : public base_ZeroCrossings
{
public:
    ZeroCrossingsDiff();

    /**
     * @brief Calculate forward difference for detecting zero crossings
     * @param[in] input of floats
     * @param[out] dst
     */
    void operator ()(const cv::Mat1f &src, cv::Mat1f &dst) const;

private:
    typedef cv::Mat1f::const_iterator Mat1fCIter; ///< Convinience typedef for iterating through matrix of floats
};

#endif // SEM_CORE_ZEROCROSSINGS_H_
