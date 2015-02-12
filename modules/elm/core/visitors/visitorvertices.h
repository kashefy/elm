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
#include "elm/core/typedefs_fwd.h"
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

    elm::VecVertices operator()(uchar c);

    elm::VecVertices operator()(const cv::Mat1f &m);

    elm::VecVertices operator()(const elm::SparseMat1f &m);

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

#endif // __WITH_PCL

} // namespace elm

#endif // _ELM_CORE_VISITORVERTICES_H_
