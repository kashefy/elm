#ifndef ELM_CORE_VISITORVERTICES_H_
#define ELM_CORE_VISITORVERTICES_H_

#ifdef __WITH_PCL // the following visitor derived class definitions requires PCL support

#include "sem/core/pcl/typedefs_fwd.h"
#include "sem/core/pcl/vertices.h"
#include "sem/core/typedefs_fwd.h"
#include "sem/core/visitors/visitor_.h"

/**
 * @brief visitor class for converting to STL vector of PCL vertices
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorVecVertices :
        public Visitor_<sem::VecVertices >
{
public:
    void Reset();

    elm::VecVertices operator()(const elm::VecVertices &vv);

    elm::VecVertices operator()(sem::CloudXYZPtr &c);

    elm::VecVertices operator()(sem::CloudNrmlPtr &c);

    elm::VecVertices operator()(sem::CloudPtNrmlPtr &c);

    elm::VecVertices operator()(float f);

    elm::VecVertices operator()(int n);

    elm::VecVertices operator()(uchar c);

    elm::VecVertices operator()(const elm::Mat_f &m);

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

#else // __WITH_PCL
    #warning "Unable to define VisitorVecVertices visitor without PCL support."
#endif // __WITH_PCL


#endif // ELM_CORE_VISITORVERTICES_H_
