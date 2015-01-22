/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_ENCODING_INTENSITYCONTRAST_H_
#define ELM_ENCODING_INTENSITYCONTRAST_H_

#include <memory>

#include <opencv2/core.hpp>

#include "elm/encoding/ganglion.h"

/**
 * @brief base class for measuring intensity contrast
 */
class base_IntensityContrast
{
public:
    virtual ~base_IntensityContrast();

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
 * @brief Implement intensity constrast measure using a model of retinal ganglion cells
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

#endif // ELM_ENCODING_INTENSITYCONTRAST_H_
