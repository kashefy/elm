#include "core/hist.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"

using namespace cv;

Hist1Ch::~Hist1Ch()
{
}

Hist1Ch::Hist1Ch()
{
    Reconfigure(10, std::make_pair<float, float>(0., 255.), true, false);
}

//TODO: throw when size==0?
void Hist1Ch::Reconfigure(int size, const std::pair<float, float> &range, bool do_uniform, bool do_accumulate)
{
    if(size < 0) { SEM_THROW_VALUE_ERROR("size must be >= 0"); }
    size_ = size;

    range_ = range;

    do_uniform_ = do_uniform;
    do_accumulate_ = do_accumulate;
}

// TODO: Find out why some Mat types don't work.
Mat1f Hist1Ch::Compute(const cv::Mat &in, InputArray mask)
{
    if(in.channels() != 1) {

        std::stringstream s;
        s << "Only single-channel input supported for now for histogram calculcation.";
        s << "Encountered input with " << in.channels() << " channels.";
        SEM_THROW_BAD_DIMS(s.str());
    }

    uchar depth = in.type() & CV_MAT_DEPTH_MASK;
    switch ( depth ) {
    case CV_8S:  SEM_THROW_TYPE_ERROR("Caluclating histogram from CV_8S matrices not yet supported"); break;
    case CV_16S:  SEM_THROW_TYPE_ERROR("Caluclating histogram from CV_16S matrices not yet supported"); break;
    case CV_32S:  SEM_THROW_TYPE_ERROR("Caluclating histogram from CV_32S matrices not yet supported"); break;
    case CV_64F:  SEM_THROW_TYPE_ERROR("Caluclating histogram from CV_64F matrices not yet supported"); break;
      default: break;
    }

    /// Set the ranges
    float range[2] = { range_.first, range_.second } ;
    const float* hist_range = { range };

    calcHist(&in, 1, 0, mask, hist_, 1, &size_, &hist_range,
             do_uniform_, do_accumulate_);

    return hist_.t();
}
