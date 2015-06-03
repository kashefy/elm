/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_CONCATENTATECLOUDXYZANDNORMAL_H_
#define _ELM_LAYERS_CONCATENTATECLOUDXYZANDNORMAL_H_

#ifndef __WITH_PCL
    // Disabling ConcatentateCloudXYZAndNormal layer due to no PCL support and defining it as a non-supported
    #include "elm/layers/layernotsupported.h"
namespace elm {
    class ConcatentateCloudXYZAndNormal : public ELM_LAYER_NOT_SUPPORTED(ConcatentateCloudXYZAndNormal, "Building with PCL is required for supporting ConcatentateCloudXYZAndNormal layer.");
} // namespace elm
#else   // __WITH_PCL

#include <string>

#include <boost/shared_ptr.hpp>

#include "elm/core/pcl/typedefs_fwd.h"
#include "elm/core/base_Layer.h"

namespace elm {

class ConcatentateCloudXYZAndNormal : public base_Layer
{
public:
    static const std::string KEY_INPUT_XYZ;     ///< key to xyz cloud
    static const std::string KEY_INPUT_NORMAL;  ///< key to normal cloud
    static const std::string KEY_OUTPUT_POINT_NORMAL;        ///< key output

    ConcatentateCloudXYZAndNormal();

    void Clear();

    void Reconfigure(const LayerConfig &cfg);

    void InputNames(const LayerInputNames &in_names);

    void OutputNames(const LayerOutputNames &out_names);

    void Activate(const Signal &signal);

    void Response(Signal &signal);

protected:
    std::string name_xyz_;      ///< name of xyz input in signal object
    std::string name_norml_;    ///< name of normal input in signal object
    std::string name_out_pnt_nrml_; ///< desitnation name of output cloud

    CloudPtNrmlPtr dst_cloud_;
};

} // namespace elm

#endif   // __WITH_PCL

#endif // _ELM_LAYERS_CONCATENTATECLOUDXYZANDNORMAL_H_
