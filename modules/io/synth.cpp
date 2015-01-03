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
    nb_variations_ = nb_variations;
    delta_ = 180.f/static_cast<float>(nb_variations_);
}

void SynthBars::Next(Mat &feature, Mat &label)
{
    unsigned int index = randu<unsigned int>()%nb_variations_;
    float angle = IndexToDeg(index);

    Draw(angle, feature);

    label = Mat1f(1, 1, angle);
}

void SynthBars::Draw(float angle_deg, Mat &img) const
{
    Mat1f label = Mat1f(2, 1, -angle_deg);
    Mat1f mag(2, 1, rows_+cols_), x, y;

    polarToCart(mag, label, x, y, true);

    Point2i centre(cols_/2, rows_/2),
            a(x(0), y(0)), b(x(1), y(1));
    a += centre;

    img = Mat1b::zeros(rows_, cols_);
    line(img, a, centre-b, Scalar_<uchar>(255), 3, LINE_8);
}

float SynthBars::IndexToDeg(unsigned int index) const
{
    return (index % nb_variations_) * delta_;
}


