/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/mat_vector_utils.h"

#include <opencv2/core.hpp>

#include "elm/core/exception.h"

using namespace std;
using namespace cv;
using namespace elm;

Mat1f elm::ElementsAt(const VecMat1f &v, int r, int c)
{
    Mat1f values(1, static_cast<int>(v.size()));
    int k=0;
    for(VecMat1f::const_iterator itr=v.begin();
        itr != v.end();
        itr++, k++) {

        if(r < 0 || r >= (*itr).rows || c< 0 || c >= (*itr).cols) {

            stringstream s;
            s << "Cannot access element at (" << r << "," << c << ") in v[" << k << "]"
              << " with dims (" << (*itr).rows << "," << (*itr).cols <<")";
            ELM_THROW_BAD_DIMS(s.str());
        }
        else {
            values(k) = (*itr)(r, c);
        }
    }

    return values;
}

Mat1f elm::Reshape(const VecMat1f &v)
{
    Mat1f dst;
    const int COLS = static_cast<int>(v.size());
    if(COLS > 0) {

        // define dimensions of desintation matrix
        Size2i s = v[0].size();
        const int ROWS = static_cast<int>(s.area());

        // avoid defining an n-col, zero-row destination matrix
        // ensure dimensions of destination matrix are all non-zero
        if(ROWS > 0) {
            dst = Mat1f(ROWS, COLS); // initialize destination matrix

            // copy elements
            // at r,c of each matrix inside vector
            // to row k of dst matrix
            int k = 0;
            for(int r=0; r<s.height; r++) {

                for(int c=0; c<s.width; c++) {

                    ElementsAt(v, r, c).copyTo(dst.row(k++));
                }
            }
        }
    }
    // else { nothing to do }

    return dst;
}

int elm::tril(const Mat1f &src, vector<Mat1f > &dst)
{
    int nb_ab = 0;

    if(!src.empty()) {

        dst.clear();
        dst.reserve(src.rows);

        for(int r=1; r<src.rows; r++) {

            dst.push_back(src.row(r).colRange(0, r));
            nb_ab += r;
        }
    }

    return nb_ab;
}
