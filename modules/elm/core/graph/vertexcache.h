#ifndef _ELM_GRAPH_VERTEXCACHE_H_
#define _ELM_GRAPH_VERTEXCACHE_H_

#include "elm/core/graph/graphattr_impl_typedefs.h"

namespace elm {

/**
 * @brief class for Vertex Id and descriptor caching and recording of subtitutions
 * @todo explicit test coverage, now covered through integration in GraphAttr
 */
class VertexCache
{
public:
    virtual ~VertexCache();

    VertexCache();

    VertexCache(int capacity);

    void clear();

    /**
     * @brief reserve desired capacity post-instantiation
     * @param capacity
     */
    void reserve(int capacity);

    /**
     * @brief find decriptor for a given vertex
     * @param[in] vtx_id vertex color/id
     * @param[out] vtx_descriptor
     * @return true on found
     */
    bool find(VtxColor vtx_id, VtxDescriptor &vtx_descriptor) const;

    /**
     * @brief check if a vertex exists
     * @param vtx_id vertex color/id
     * @return true if exists in cache and is not masked
     */
    bool exists(VtxColor vtx_id) const;

    /**
     * @brief insert vertex into cache
     * @param vtx_id
     * @param vtx_descriptor
     */
    void insert(VtxColor vtx_id, const VtxDescriptor &vtx_descriptor);

    /**
     * @brief have cahce reflect vertex substitution
     * @param src
     * @param dst
     */
    void recordSubstitution(VtxColor src, VtxColor dst);

    /**
     * @brief remove/mask vertex in cache
     * @param vtx_id
     */
    void remove(VtxColor vtx_id);

    /**
     * @brief Get true Id for a vector (in case it got substituted along the way)
     * @param vtx_id
     * @return true id
     */
    VtxColor Id(VtxColor vtx_id) const;

protected:
    // members
    std::vector<VtxDescriptor > descriptors_;   ///< cache vertex descriptors
    std::vector<VtxColor > ids_;                ///< cached vertex ids/colors
    int lim_;                                   ///< limit of meaningful entries in cache
    int capacity_;                              ///< cache capacity
};

} // namespace elm

#endif // _ELM_GRAPH_VERTEXCACHE_H_
