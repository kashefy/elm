/**@file Synthetic data generator
  */
#ifndef SEM_IO_SYNTH_H_
#define SEM_IO_SYNTH_H_

#include "sem/core/cv/typedefs_fwd.h"

/**
 * @brief Base class for synthetic data generation
 */
class base_Synth
{
public:
    ~base_Synth();

    /**
     * @brief Get next synthesized data point
     * @param[out] feature
     * @param[out] label
     */
    virtual void Next(cv::Mat &feature, cv::Mat &label) = 0;

protected:
    base_Synth();
};

/**
 * @brief Generate oriented bar stimuli
 */
class SynthBars : public base_Synth
{
public:
    SynthBars();

    /**
     * @brief Set Bar parameters
     * @param rows no. of rows
     * @param cols no. of columns
     * @param nb_variations: no. of variations, yields bars oriented at [0, 1/v, 2/v 180) degrees
     */
    void Reset(int rows, int cols, int nb_variations);

    /**
     * @brief Next oriented bar
     * @param[out] oriented bar image
     * @param[out] bar angle in degrees (0 for horizontal, 90 for vertical, 45 for /, 135 for \\)
     */
    virtual void Next(cv::Mat &feature, cv::Mat &label);

    /**
     * @brief Draw oriented bar image
     * @param angle_deg[in] angle in degrees
     * @param[out] image with oriented bar
     */
    virtual void Draw(float angle_deg, cv::Mat &img) const;

    /**
     * @brief Calculate angle from index integer
     * @return angle [deg]
     */
    float IndexToDeg(unsigned int index) const;

protected:
    int rows_;      ///< no. of rows in bar image
    int cols_;      ///< no. of columns in bar image
    int nb_variations_;         ///< no. of orientation variations
    float delta_;   ///< no. of orientation variations
};

#endif // SEM_IO_SYNTH_H_
