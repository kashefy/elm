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

    virtual void operator ()(cv::InputArray src, cv::OutputArray dst) const = 0;

protected:
    base_ZeroCrossings();
};

class ZeroCrossingsSobel : public base_ZeroCrossings
{
public:
    ZeroCrossingsSobel();

    void operator ()(cv::InputArray src, cv::OutputArray dst) const;

private:
    typedef cv::Mat1f::const_iterator Mat1fCIter;
};

#endif // SEM_CORE_ZEROCROSSINGS_H_
