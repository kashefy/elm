#include "core/cv/neighborhood.h"

#include "core/exception.h"

using namespace cv;

void sem::NeighMeanVar(const Mat1f &m, int radius, Mat1f &neigh_mean, Mat1f &neigh_var,
                       int border_type, const Scalar &value)
{
    int diameter = radius*2+1;

    // check radius
    if(radius < 0) {
        SEM_THROW_VALUE_ERROR("radius must be >= 0");
    }
    else if(radius == 0) {

        // we can short-circuit the resuls for this case
        neigh_mean = m.clone();
        neigh_var = Mat1f::zeros(m.size());
    }
    else {
        neigh_mean = Mat1f(m.size());
        neigh_var = Mat1f(m.size());

        // pad borders
        Mat1f padded;
        copyMakeBorder(m, padded,
                       radius, radius, radius, radius,
                       border_type, value);

        for(int r=0; r<m.rows; r++) {

            Mat1f sub_rows = padded.rowRange(r, r+diameter);

            for(int c=0; c<m.cols; c++) {

                Mat sub_mean, sub_stddev;
                meanStdDev(sub_rows.colRange(c, c+diameter),
                           sub_mean, sub_stddev);

                neigh_mean(r, c) = static_cast<float>(sub_mean.at<double>(0));
                neigh_var(r, c) = static_cast<float>(sub_stddev.at<double>(0)); // don't forget to square
            }
        }
        multiply(neigh_var, neigh_var, neigh_var);
    }
}
