#ifndef _ELM_GRAPH_VERTEXCACHE_H_
#define _ELM_GRAPH_VERTEXCACHE_H_

#include "elm/core/graph/graphattr_impl_typedefs.h"

namespace elm {

/**
 * @brief class for Vertex Id and descriptor caching and recording of subtitutions
 *
 * Every entry can have the following states:
 * 1. masked == never entered == removed
 * 2. substituted == linked
 * 3. true == identity
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


    /**
     * @brief check if key is valid
     * Does not determine if key is a valid vertex id
     * @param key
     * @return true if key is valid
     */
    bool validKey(VtxColor key) const;

    /**
     * @brief check if a vertex cache just links to another entry
     * Useful for differentiating true vertex from substituted one
     * @param vtx_id vertex color/id
     * @return true if vertex is just a link to another
     */
    bool isLink(VtxColor vtx_id) const;

    /**
     * @brief check if a vertex is masked
     * Does not determine if vertex is linked to another
     * @param vtx_id vertex color/id
     * @return true if vertex is masked
     */
    bool isMasked(VtxColor vtx_id) const;

protected:

    bool isLinkValue(VtxColor key, VtxColor value) const;

    bool isMaskedValue(VtxColor vtx_id_value) const;

    // members
    std::vector<VtxDescriptor > descriptors_;   ///< cache vertex descriptors
    std::vector<VtxColor > ids_;                ///< cached vertex ids/colors
    int lim_;                                   ///< limit of meaningful entries in cache
    int capacity_;                              ///< cache capacity
};

} // namespace elm

#endif // _ELM_GRAPH_VERTEXCACHE_H_
