/**@file Synthetic data generator
  */
#ifndef SEM_IO_SYNTH_H_
#define SEM_IO_SYNTH_H_

#include <opencv2/core.hpp>

/**
 * @brief Base class for synthetic data generation
 */
class base_Synth
{
public:
    ~base_Synth();

    /**
     * @brief Get next synthesized data point
     * @return synthesized data
     */
    virtual cv::Mat Next() = 0;

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
     * @param nb_variations: no. of variations, yields bars oriented at [0, 1/v, 2/v 180] degrees
     */
    void Reset(int rows, int cols, int nb_variations);

    /**
     * @brief Next oriented bar
     * @return oriented bar image
     */
    cv::Mat Next();

    /**
     * @brief Calculate angle from index integer
     * @return angle [deg]
     */
    float IndexToDeg(int index) const;

private:
    int rows_;      ///< no. of rows in bar image
    int cols_;      ///< no. of columns in bar image
    int n_;         ///< no. of orientation variations
    float delta_;   ///< no. of orientation variations
};

#endif // SEM_IO_SYNTH_H_
