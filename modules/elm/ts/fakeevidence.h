/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_TS_FAKEEVIDENCE_H_
#define ELM_TS_FAKEEVIDENCE_H_

#include <opencv2/core.hpp>

/**
 * @brief Class for generating one dimensional fake evidence/feature vectors
 */
class FakeEvidence
{
public:
    /**
     * @brief Construct object for generating fake evidence vector
     * @param no. of features (translates directly to feature vector size)
     */
    FakeEvidence(int nb_features);

    /**
     * @brief get next fake feature vector
     *
     * if state is even then [0, 1, 0, ...]
     * else if state is odd then [1, 0, 1, ...]
     *
     * @param state to see vector generation
     * @return feature vector
     */
    cv::Mat next(int state);

    int nb_features_;
};

#endif // ELM_TS_FAKEEVIDENCE_H_
