#include "elm/core/graph/base_GraphVertexOp.h"
#include "elm/core/debug_utils.h"
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace elm;

boost::mutex base_GraphVertexOp::mtx_;

base_GraphVertexOp::~base_GraphVertexOp()
{
}

base_GraphVertexOp::base_GraphVertexOp()
{
}

void base_GraphVertexOp::mutableOpCaller(const Mat1i &img, const Mat1b &mask, Mat1f &dst)
{
//    ELM_COUT_VAR(""<<img);
//    ELM_COUT_VAR(""<<mask);
    dst = mutableOp(img, mask);
}
