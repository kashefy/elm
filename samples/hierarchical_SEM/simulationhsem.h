#ifndef SIMULATIONHSEM_H_
#define SIMULATIONHSEM_H_

#include <iostream>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "core/base_Layer.h"
#include "core/typedefs.h"

/**
 * @brief class for simulating hierarchical spike-based Expectation Maximization
 * An extension of the SEM algorithm as seen in @cite Nessler2010
 */
class SimulationHSEM
{
public:
    SimulationHSEM();

    void Learn();

    void Test();

    void Eval();

protected:
    // static members
    static const std::string NAME_STIMULUS;
    static const std::string NAME_POP_CODE;
    static const std::string NAME_SPIKES_Y;
    static const std::string NAME_SPIKES_Z;
    static const std::string NAME_WEIGHTS;

    // methods
    std::shared_ptr<base_Layer> InitPopulationCode() const;

    std::shared_ptr<base_Layer> InitLayerY() const;

    std::shared_ptr<base_Layer> InitLearners(int nb_features, int history_length) const;

    void VisualizeOnOffWeights(const cv::Mat1f &weights);

    // members
    std::shared_ptr<base_Layer> pop_code_;
    std::shared_ptr<base_Layer> y_;
    std::shared_ptr<base_Layer> z_;
    size_t nb_learners_;   ///< no. of learners (e.g. ZNeuron)

};
#endif // SIMULATIONHSEM_H_
