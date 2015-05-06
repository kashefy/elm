/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_BASE_STATEFULPOPULATIONCODE_H_
#define _ELM_ENCODING_BASE_STATEFULPOPULATIONCODE_H_

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"
#include "elm/core/typedefs_sfwd.h"
#include "elm/encoding/base_populationcode.h"

namespace elm {

/**
 * @brief a base class for population codes that are stateful
 */
class base_StatefulPopulationCode : public base_PopulationCode
{
public:
    static const std::string KEY_OUTPUT_OPT_STATE;  ///< optional output for state due to most recent stimuli

protected:
    virtual ~base_StatefulPopulationCode();

    virtual void Clear();

    virtual void OutputNames(const LayerIONames &config);

    virtual void Response(Signal &signal);

    base_StatefulPopulationCode();

    // members
    OptS name_state_;   ///< optional destination name for state in signal object

    cv::Mat1f state_;   ///< internal state due to most recent stimuli
};

} // namespace elm

#endif // _ELM_ENCODING_BASE_STATEFULPOPULATIONCODE_H_
