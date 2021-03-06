/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_VISITORVERTICES_H_
#define _ELM_CORE_VISITORVERTICES_H_

#ifdef __WITH_PCL // the following visitor derived class definitions requires PCL support

#include "elm/core/pcl/typedefs_fwd.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/typedefs_sfwd.h"
#include "elm/core/visitors/visitor_.h"

namespace elm {

/**
 * @brief visitor class for converting to STL vector of PCL vertices
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorVecVertices :
        public Visitor_<elm::VecVertices >
{
public:
    void Reset();

    elm::VecVertices operator()(const elm::VecVertices &vv);

    elm::VecVertices operator()(elm::CloudXYZPtr &c);

    elm::VecVertices operator()(elm::CloudNrmlPtr &c);

    elm::VecVertices operator()(elm::CloudPtNrmlPtr &c);

    elm::VecVertices operator()(float f);

    elm::VecVertices operator()(int n);

    elm::VecVertices operator()(pod::uchar c);

    elm::VecVertices operator()(const cv::Mat1f &m);

    elm::VecVertices operator()(const elm::SparseMat1f &m);

    /**
     * @brief elm::VecMat1f to elm::VecVertices
     * Mat1f elements will be flattened.
     */
    elm::VecVertices operator()(const elm::VecMat1f &v);

protected:
    template <typename TScalar>
    void FromScalar(TScalar s) {

        Reset();
        pcl::Vertices v;
        v.vertices.push_back(static_cast<uint32_t>(s));
        vv_.push_back(v);
    }

    elm::VecVertices vv_;   ///< internal copy for caching most recent conversion result
};

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_CORE_VISITORVERTICES_H_
