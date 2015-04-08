/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matio_utils.h"

#ifdef __WITH_MATIO

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"

#include "elm/core/debug_utils.h"
#include "elm/core/cv/mat_vector_utils.h"
#include "elm/core/cv/mat_vector_utils_inl.h"

using namespace cv;

unsigned int elm::MATIOClassTOCV_TYPE(matio_classes type)
{
    unsigned int cv_type;
    switch(type) {

    case MAT_C_CHAR:
        cv_type = CV_8S; // or unsigned?
        break;
    case MAT_C_DOUBLE:
        cv_type = CV_64F;
        break;
    case MAT_C_UINT8:
        cv_type = CV_8U;
        break;
    case MAT_C_UINT16:
        cv_type = CV_16U;
        break;
    case MAT_C_INT8:
        cv_type = CV_8S;
        break;
    case MAT_C_INT16:
        cv_type = CV_16S;
        break;
    case MAT_C_INT32:
        cv_type = CV_32S;
        break;
    case MAT_C_SINGLE:
        cv_type = CV_32F;
        break;

    default:
        ELM_THROW_TYPE_ERROR("unrecognized type");
    }

    return cv_type;
}

void elm::SliceCopy(const cv::Mat &src, int dim, int idx, cv::Mat& dst)
{
    int *sizes = new int[dim];

    int *src_sizes_rev = new int[src.dims];
    for(int i=0; i<src.dims; i++) {

        src_sizes_rev[i] = src.size[src.dims-i-1];
    }

    for(int i=0; i<dim; i++) {

        sizes[i] = src.size[i];
    }

    dst = Mat(dim, sizes, src.type());

    const int DIM_SUB_IDX = src.dims-dim-1;

    int l_start, l_end;
    {
        int *subs_start = new int[src.dims];
        int *subs_end = new int[src.dims];
        for(int d=0; d<src.dims; d++) {

            if(d==DIM_SUB_IDX) {

                subs_start[d] = subs_end[d] = idx;
            }
            else {

                subs_start[d] = 0;
                subs_end[d] = src_sizes_rev[d]-1;
            }
        }

        l_start = sub2ind(src.dims, src_sizes_rev, subs_start);
        l_end = sub2ind(src.dims, src_sizes_rev, subs_end);

        delete []subs_start;
        delete []subs_end;
    }

    const size_t ELEM_SIZE = src.elemSize();
    uchar *src_data_ptr = src.data + l_start * ELEM_SIZE;
    uchar *dst_data_ptr = dst.data;

    for(int l=l_start; l<=l_end; l++) {

        //std::vector<int> subs;
        //ind2sub(l, src.dims, src_sizes_rev, subs);
        //std::cout<<l<<" "<<elm::to_string(subs)<<" "<< (subs[DIM_SUB_IDX] == idx) << " "<<std::endl;
        //if(subs[DIM_SUB_IDX] == idx) {

            memcpy(dst_data_ptr, src_data_ptr, ELEM_SIZE);
            dst_data_ptr += ELEM_SIZE;
        //}

        src_data_ptr += ELEM_SIZE;
    }

    delete [] sizes;
    delete [] src_sizes_rev;
}

void elm::ind2sub(int idx, int dims, const int *sizes, std::vector<int> &subs)
{
    subs.resize(dims);

    int *prod = new int [dims];
    for (int i=0; i < dims; i++) {

        prod[i] = 1;
        for (int j=dims-1; j > i; j--) {

            prod[i] *= sizes[j];
        }
    }

    for (int i=0; i < dims; i++) {

        subs[i] = idx;
        for (int j=0; j < i ; j++) {

            subs[i] = subs[i] % prod[j];
        }
        subs[i] = static_cast<int>(floor( static_cast<float>(subs[i] / prod[i]) ));
    }

    delete [] prod;
}

int elm::sub2ind(int dims, const int *sizes, const int *subs)
{
    int idx = 0;
    for (int i=0; i < dims; i++) {

        int prod = 1;
        for (int j=dims-1; j > i; j--) {

            prod *= sizes[j];
        }
        idx += subs[i] * prod;
    }
    return idx;
}

void elm::Mat3DTo3Ch(const Mat &src, Mat &dst)
{
    const int ROWS=src.size[0];
    const int COLS=src.size[1];
    const int NB_CHANNELS=src.size[2];

    dst = Mat(ROWS, COLS, CV_MAKETYPE(src.type(), NB_CHANNELS));

    int i=0;
    for(int ch=0; ch<NB_CHANNELS; ch++) {

        for(int c=0; c<COLS; c++) {

            for(int r=0; r<ROWS; r++) {

                dst.at<Vec3b>(r,c)[ch] = src.at<uchar>(i);
                i++;
            }
        }
    }
}

#endif // __WITH_MATIO
