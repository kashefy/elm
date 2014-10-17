#include "io/synth.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"

using namespace cv;

base_Synth::base_Synth()
{
}

base_Synth::~base_Synth()
{
}

SynthBars::SynthBars()
    : base_Synth()
{
    Reset(28, 28, 6);
}

void SynthBars::Reset(int rows, int cols, int nb_variations)
{
    if(nb_variations <= 0) { SEM_THROW_VALUE_ERROR("No. of variation must be > 0."); }
    rows_ = rows;
    cols_ = cols;
    n_ = nb_variations;
    delta_ = 180.f/static_cast<float>(n_);
}

Mat SynthBars::Next()
{
    Point2i centre(cols_/2, rows_/2);
    Mat1f deg = Mat1f(1, 1, -IndexToDeg(randu<int>()%n_));
    Mat1f mag(1, 1, rows_+cols_), x, y;

    polarToCart(mag, deg, x, y, true);

    Point2i a(x(0), y(0));
    a += centre;

    polarToCart(-mag, deg, x, y, true);

    Point2i b(x(0), y(0));
    b += centre;

    Mat1b img = Mat1b::zeros(rows_, cols_);
    line(img, a, b, Scalar_<uchar>(255), 5, LINE_8);

    return img;
}

float SynthBars::IndexToDeg(int index) const
{
    return index * delta_;
}


