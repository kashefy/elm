#include "elm/core/cv/lut.h"

#include <opencv2/core/core.hpp>

using namespace elm;

LUT::~LUT()
{

}

LUT::LUT()
    : count_(0),
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

    for (int i=0; i<=max_entry_; i++) {

        if(table_[i] == max_) {

            table_[i] = min_;
        }
    }
    return min_;
}

void LUT::apply(cv::Mat1i &m) const
{
    if(table_.empty()) {

        return;
    }

    for(size_t i=0; i<m.total(); i++) {

        m(i) = table_[m(i)];
    }
}
