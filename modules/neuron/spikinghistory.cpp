#include "neuron/spikinghistory.h"

SpikingHistory::SpikingHistory(int dims, int len)
    : len_(len),
      dims_(dims)
{
    Reset();
}

void SpikingHistory::Advance()
{
    cv::Mat mask_rows = history_ != 0;
    cv::subtract(history_,
                 MatI::ones(history_.rows, history_.cols),
                 history_,
                 mask_rows);
}

MatI SpikingHistory::History() const
{
    return history_;
}

void SpikingHistory::Reset()
{
    history_ = MatI::zeros(1, dims_)+len_;
}

bool SpikingHistory::Recent(int index) const
{
    return history_(index) > 0;
}

void SpikingHistory::Update(const cv::Mat &spike_mask)
{
    history_.setTo(len_, spike_mask);
}


