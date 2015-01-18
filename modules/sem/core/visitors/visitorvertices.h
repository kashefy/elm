#ifndef SEM_CORE_VISITORVERTICES_H_
#define SEM_CORE_VISITORVERTICES_H_

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

    sem::VecVertices operator()(const sem::VecVertices &vv);

    sem::VecVertices operator()(sem::CloudXYZPtr &c);

    sem::VecVertices operator()(float f);

    sem::VecVertices operator()(int n);

    sem::VecVertices operator()(uchar c);

    sem::VecVertices operator()(const sem::Mat_f &m);

protected:
    template <typename TScalar>
    void FromScalar(TScalar s) {

        Reset();
        pcl::Vertices v;
        v.vertices.push_back(static_cast<uint32_t>(s));
        vv_.push_back(v);
    }

    sem::VecVertices vv_;   ///< internal copy for caching most recent conversion result
};

#else // __WITH_PCL
    #warning "Unable to define VisitorVecVertices visitor without PCL support."
#endif // __WITH_PCL


#endif // SEM_CORE_VISITORVERTICES_H_
