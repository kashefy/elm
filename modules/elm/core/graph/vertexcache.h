#ifndef _ELM_GRAPH_VERTEXCACHE_H_
#define _ELM_GRAPH_VERTEXCACHE_H_

#include "elm/core/graph/graphattr_impl_typedefs.h"

namespace elm {

/**
 * @brief class for Vertex Id and descriptor caching and recording of subtitutions
 */
class VertexCache
{
public:
    virtual ~VertexCache();

    VertexCache();

    VertexCache(int capacity);

    void clear();

    void reserve(int capacity);

    bool find(VtxColor vtx_id, VtxDescriptor &vtx_descriptor) const;

    void insert(VtxColor vtx_id, const VtxDescriptor &vtx_descriptor);

    void recordSubstitution(VtxColor src, VtxColor dst);

protected:
    // members
    std::vector<VtxDescriptor > descriptors_;   ///< cache vertex descriptors
    std::vector<VtxColor > ids_;
    int lim_;
};

} // namespace elm

#endif // _ELM_GRAPH_VERTEXCACHE_H_
