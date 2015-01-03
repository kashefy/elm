#include "neuron/spikinghistory.h"

using namespace cv;

SpikingHistory::SpikingHistory(int dims, int len)
    : len_(len),
      dims_(dims)
{
    Reset();
}

void SpikingHistory::Advance()
{
    subtract(history_, 1, history_, history_ > 0);
}

Mat1i SpikingHistory::History() const
{
    return history_;
}

void SpikingHistory::Reset()
{
    history_ = Mat1i::zeros(1, dims_);
}

void SpikingHistory::Reset(int index)
{
    history_(index) = 0;
}

bool SpikingHistory::Recent(int index) const
{
    return history_(index) > 0;
}

cv::Mat SpikingHistory::Recent() const
{
    return history_ > 0;
}

void SpikingHistory::Update(const cv::Mat &spike_mask)
{
    history_.setTo(len_, spike_mask);
}


