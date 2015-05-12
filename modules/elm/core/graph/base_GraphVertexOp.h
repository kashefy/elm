/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_
#define _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_

#include <boost/thread/mutex.hpp>

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

    virtual void mutableOpCaller(const cv::Mat1i& img, const cv::Mat &mask, cv::Mat1f &dst);

    /**
     * @brief operator () to pass for applying to graph vertex
     * @param img input image
     * @param mask masked (usually by vertex id)
     * @return transformation result
     */
    virtual cv::Mat1f mutableOp(const cv::Mat1i& img, const cv::Mat &mask) = 0;

protected:
    base_GraphVertexOp();

    static boost::mutex mtx_;  ///< mutex variable for keeping mutableOp thread safe
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_BASE_GRAPHVERTEXOP_H_
