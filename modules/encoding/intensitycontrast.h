#ifndef SEM_ENCODING_INTENSITYCONTRAST_H_
#define SEM_ENCODING_INTENSITYCONTRAST_H_

#include <memory>

#include <opencv2/core.hpp>

#include "encoding/ganglion.h"

/**
 * @brief base class for measuring intensity contrast
 */
class base_IntensityContrast
{
public:
    ~base_IntensityContrast();

    /**
     * @brief Initialize
     * @param radius in pixels
     * @param filter scale
     */
    virtual void Init(int radius, float scale) = 0;

    /**
     * @brief Compute intensity constrast from stimulus
     * @param stimulus
     */
    virtual void Compute(cv::InputArray stimulus) = 0;

    /**
     * @brief Get Response
     * @return response from most recent stimulus
     */
    virtual cv::Mat Response() = 0;

protected:
    base_IntensityContrast();
};

/**
 * @brief Implement intensity constrast measure using model retinal ganglion cells
 */
class RetGang : public base_IntensityContrast
{
public:
    RetGang();

    void Init(int radius, float scale);

    void Compute(cv::InputArray stimulus);

    cv::Mat Response();

    virtual cv::Mat State() const;

private:

    std::unique_ptr<base_Ganglion> rg_; /// retinal ganglion cells model

    cv::Mat1f state_;
};

#endif // SEM_ENCODING_INTENSITYCONTRAST_H_
