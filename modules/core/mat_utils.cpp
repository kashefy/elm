#include "core/mat_utils.h"

#include <iostream>

void sem::CumSum(const MatF &src, MatF &dst)
{
    if(dst.total() < src.total() && src.total() > 0) {

        dst = MatF::zeros(src.rows, src.cols);
    }

    float interm_sum = 0;
    for(unsigned int i = 0; i < src.total(); i++) {

        interm_sum += src(i);
        dst(i) = interm_sum;
    }
}
