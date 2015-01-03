#include "neuron/spikinghistory.h"

using namespace cv;

SpikingHistory::SpikingHistory(int dims, int len)
    : len_(len),
      dims_(dims),
      history_(1, dims_, 0)
{
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
    history_.setTo(0);
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

SpikingHistory SpikingHistory::ColRange(int start, int end) const
{
    SpikingHistory obj(dims_, len_);
    obj.History(history_.colRange(start, end));
    return obj;
}

void SpikingHistory::History(const Mat1i &h)
{
    history_ = h;
}


