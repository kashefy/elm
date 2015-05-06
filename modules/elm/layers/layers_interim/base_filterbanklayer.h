/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file Define classes for encapsulating, applying filter banks
 */
#ifndef _ELM_LAYERS_LAYERS_INTERIM_BASE_FILTERBANKLAYER_H_
#define _ELM_LAYERS_LAYERS_INTERIM_BASE_FILTERBANKLAYER_H_

#include <opencv2/core/core.hpp>    // complete definition for VecMat1f

#include "elm/core/exception.h"
#include "elm/core/typedefs_sfwd.h"
#include "elm/encoding/base_filterbank.h"
#include "elm/layers/layers_interim/base_singleinputfeaturelayer.h"

namespace elm {

namespace detail {

const std::string BASE_FILTERBANK_LAYER__KEY_OUTPUT_RESPONSE = "m";   ///< define string here to ensure early initialization for test purposes

}

/**
 * @brief base class for filter banks
 *  The interface should aid in iterating through response
 */
class base_FilterBankLayer :
        public base_FilterBank,
        public base_SingleInputFeatureLayer
{
public:
    static const std::string KEY_OUTPUT_RESPONSE;   ///< key to output/response

    /** Empty default destructor
     */
    virtual ~base_FilterBankLayer();

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    /**
     * @brief Empty default constructor, only accessible by child classes
     */
    base_FilterBankLayer();

    std::string name_out_;  ///< desintation name for response in signal object

};

} // namespace elm

#endif // _ELM_LAYERS_LAYERS_INTERIM_BASE_FILTERBANKLAYER_H_
