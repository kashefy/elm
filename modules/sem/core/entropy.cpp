#include "sem/core/entropy.h"

#include "sem/core/exception.h"

using namespace cv;

float sem::CondEntropy(InputArray pdf)
{
    if(countNonZero(pdf) < static_cast<int>(pdf.total())) {
        SEM_THROW_VALUE_ERROR("Failed to compute cond. entropy with zero in prob. vector");
    }

    if(pdf.total() == 0) { return 0.f; }

    Mat1f prod, lg;
    log(pdf, lg);
    multiply(pdf, lg, prod);
    return -sum(prod)(0);
}
