/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/vertices.h"

#ifdef __WITH_PCL

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

VecVertices elm::Mat2VecVertices(const Mat &m)
{
    VecVertices vv;

    if(!m.empty()) {

        vv.reserve(m.rows);

        int nb_channels = m.channels();
        int len_vertices;
        if(nb_channels==1) {

            // single-channel matrix, treat columns as vertices values
            len_vertices = m.cols;
        }
        else {
            // multi-channel matrix, treat channels as vertices values
            if(m.cols > 1) {

                ELM_THROW_BAD_DIMS("Cannot extract vertices from multi-channel multi-column matrix.");
            }

            len_vertices = nb_channels;
        }

        for(int i=0; i<m.rows; i++) {

            Vertices v;
            v.vertices.reserve(size_t(len_vertices));
            for(int j=0; j<len_vertices; j++) {

                v.vertices.push_back(m.at<float>(i, j));
            }
            vv.push_back(v);
        }
    }

    return vv;
}

Mat1f elm::VecVertices2Mat(const VecVertices& vv, bool do_row_mat)
{
    Mat1f m;
    if(vv.size() > 0) {

        int nb_vertices = static_cast<int>(vv.size());
        int sz_vertex = static_cast<int>(vv[0].vertices.size());

        int mat_rows = 1;
        int mat_cols = sz_vertex;

        if(do_row_mat) {
            mat_cols *= nb_vertices;
        }
        else {

            mat_rows = nb_vertices;
        }

        m = Mat1f(mat_rows, mat_cols);
        int k=0;
        for(int i=0; i<nb_vertices; i++) {

            vector<uint32_t> tmp = vv[i].vertices;
            for(int j=0; j<sz_vertex; j++) {

                m(k++) = static_cast<float>(tmp[j]);
            }
        }
    }

    return m;
}

#endif // __WITH_PCL

