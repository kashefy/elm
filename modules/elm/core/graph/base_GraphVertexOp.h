#ifndef _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_
#define _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_

#include "elm/core/typedefs_fwd.h"

namespace elm {

/**
 * @brief Interface for defining operators
 * to use with
 * elm::GraphAttr::applyVertexToMap()
 * and
 * elm::GraphAttr::applyVerticesToMap()
 *
 */
class base_GraphVertexOp
{
public:
    virtual ~base_GraphVertexOp();

    /**
     * @brief operator () to pass for applying to graph vertex
     * @param img input image
     * @param mask masked (usually by vertex id)
     * @return transformation result
     */
    virtual cv::Mat1f operator ()(const cv::Mat1f& img, const cv::Mat1b &mask) = 0;

protected:
    base_GraphVertexOp();
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_