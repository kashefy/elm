#include "core/entropy.h"

#include "core/exception.h"

using namespace cv;

float sem::CondEntropy(const Mat1f &pdf)
{
    if(countNonZero(pdf) < static_cast<int>(pdf.total())) {
        SEM_THROW_VALUE_ERROR("Failed to compute cond. entropy with zero in prob. vector");
    }

    Mat1f prod, lg;
    log(pdf, lg);
    multiply(pdf, lg, prod);
    return -sum(prod)(0);
}
