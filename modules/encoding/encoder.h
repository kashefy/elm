#ifndef SEM_ENCODING_ENCODER_H_
#define SEM_ENCODING_ENCODER_H_

#include <memory>

#include "encoding/populationcode.h"

/**
 * @brief base class for generating spike trains from neuron state
 */
class base_Encoder
{
public:
    virtual ~base_Encoder() {}

protected:
    base_Encoder();

    std::unique_ptr<base_PopulationCode> pc_ptr_;
};

#endif // SEM_ENCODING_ENCODER_H_
