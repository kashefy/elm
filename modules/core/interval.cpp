#include "core/interval.h"

base_Interval::base_Interval(float a, float b)
    : a_(a), b_(b)
{
}

IntervalClosed::IntervalClosed(float a, float b)
    :base_Interval(a, b)
{

}

bool IntervalClosed::In(float x) const
{
    return x >= a_ && x <= b_;
}
