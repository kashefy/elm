/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/concatentatecloudxyzandnormal.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include "elm/core/featuredata.h"
#include "elm/core/inputname.h"
#include "elm/core/layerinputnames.h"
#include "elm/core/layeroutputnames.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace pcl;
using namespace elm;

extern template class pcl::PointCloud<pcl::PointXYZ >;
extern template class pcl::PointCloud<pcl::Normal >;
extern template class pcl::PointCloud<pcl::PointNormal >;

extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::Normal > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > >;

const string ConcatentateCloudXYZAndNormal::KEY_INPUT_XYZ       = "xyz";
const string ConcatentateCloudXYZAndNormal::KEY_INPUT_NORMAL    = "normal";
const string ConcatentateCloudXYZAndNormal::KEY_OUTPUT_POINT_NORMAL          = "output";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ConcatentateCloudXYZAndNormal>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(ConcatentateCloudXYZAndNormal::KEY_INPUT_XYZ)
        ELM_ADD_INPUT_PAIR(ConcatentateCloudXYZAndNormal::KEY_INPUT_NORMAL)
        ELM_ADD_OUTPUT_PAIR(ConcatentateCloudXYZAndNormal::KEY_OUTPUT_POINT_NORMAL)
        ;

ConcatentateCloudXYZAndNormal::ConcatentateCloudXYZAndNormal()
    : base_Layer()
{
}

void ConcatentateCloudXYZAndNormal::Clear()
{
    dst_cloud_.reset();
}

void ConcatentateCloudXYZAndNormal::Reconfigure(const LayerConfig &cfg)
{
}

void ConcatentateCloudXYZAndNormal::InputNames(const LayerInputNames &in_names) {

    name_xyz_ = in_names.Input(KEY_INPUT_XYZ);
    name_norml_ = in_names.Input(KEY_INPUT_NORMAL);
}

void ConcatentateCloudXYZAndNormal::OutputNames(const LayerOutputNames &out_names) {

    name_out_pnt_nrml_  = out_names.Output(KEY_OUTPUT_POINT_NORMAL);
}

void ConcatentateCloudXYZAndNormal::Activate(const Signal &signal)
{
    CloudXYZPtr xyz = signal.MostRecent(name_xyz_).get<CloudXYZPtr>();
    CloudNrmlPtr nrml = signal.MostRecent(name_norml_).get<CloudNrmlPtr>();

    // Concatenate the XYZ and normal fields
    dst_cloud_.reset(new PointCloud<PointNormal>);
    concatenateFields(*xyz, *nrml, *dst_cloud_);
}

void ConcatentateCloudXYZAndNormal::Response(Signal &signal)
{
    signal.Append(name_out_pnt_nrml_, dst_cloud_);
}

#endif // __WITH_PCL
