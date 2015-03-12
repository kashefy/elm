/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/lut.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"

using namespace elm;

LUT::~LUT()
{
}

LUT::LUT()
    : table_(1, 0),
      count_(0),
      max_entry_(0)
{
}

LUT::LUT(int capacity)
    : table_(capacity+1, 0),
      count_(0),
      max_entry_(0)
{
    table_[0] = 0;
}

void LUT::Capacity(int n)
{
    table_ = std::vector<int>(n+1);
    table_[0] = 0;
}

void LUT::insert(int lut_entry)
{
    if(static_cast<size_t>(lut_entry) >= table_.size()) {

        ELM_THROW_KEY_ERROR("LUT entry must be <= capacity.");
    }

    table_[lut_entry] = lut_entry;
    count_++;

    if(lut_entry > max_entry_) {

        max_entry_ = lut_entry;
    }
}

int LUT::update(int a, int b)
{
    int min_;
    int max_;
    if(a == b) {

        return a; // nothing to do
    }
    else if(a > b) {

        max_ = a;
        min_ = b;
    }
    else {

        max_ = b;
        min_ = a;
    }

    for (int i=0; i<=max_; i++) {

        if(table_[i] == max_) {

            table_[i] = min_;
        }
    }
    return min_;
}

void LUT::apply(cv::Mat1i &m) const
{
    if(count_ < 1) {

        return;
    }

    if(m.isContinuous()) {

        int *mat_data_ptr = reinterpret_cast<int*>(m.data);
        const int *END = reinterpret_cast<int*>(m.dataend);

        for(; mat_data_ptr != END; mat_data_ptr++) {

            *mat_data_ptr = table_[*mat_data_ptr];
        }
    }
    else {

        const int NB_ELEMENTS = static_cast<int>(m.total());
        for(int i=0; i<NB_ELEMENTS; i++) {

            m(i) = table_[m(i)];
        }
    }
}
