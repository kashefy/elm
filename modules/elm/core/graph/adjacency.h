#ifndef ELM_CORE_GRAPH_ADJACENCY_H_
#define ELM_CORE_GRAPH_ADJACENCY_H_

#include "elm/core/cv/typedefs_fwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

#ifdef __WITH_PCL

cv::Mat1f TriangulatedCloudToAdjacencyMat(const CloudXYZPtr &cld, const Triangles &t);

#endif // __WITH_PCL


} // namespace elm

#endif // ELM_CORE_GRAPH_ADJACENCY_H_
