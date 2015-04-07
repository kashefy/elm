/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matio_utils.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"

#include "elm/core/debug_utils.h"
#include "elm/core/cv/mat_vector_utils.h"
#include "elm/core/cv/mat_vector_utils_inl.h"

using namespace cv;

#ifdef __WITH_MATIO

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

    uchar *src_data_ptr = src.data;
    uchar *dst_data_ptr = dst.data;
    const size_t ELEM_SIZE = src.elemSize();

    const int DIM_SUB_IDX = src.dims-dim-1;

    for(int l=0; l<src.total(); l++) {

        std::vector<int> subs;
        ind2sub(l, src.dims, src_sizes_rev, subs);

        if(subs[DIM_SUB_IDX] == idx) {

            memcpy(dst_data_ptr, src_data_ptr, ELEM_SIZE);
            dst_data_ptr += ELEM_SIZE;
        }

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

#endif // __WITH_MATIO
