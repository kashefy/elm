/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** Assertions around common container classes
  */
#ifndef _ELM_TS_CONTAINER_H_
#define _ELM_TS_CONTAINER_H_

#include "gtest/gtest.h"

#include <vector>

#if !_MSC_VER
extern template class std::vector<std::string>;
extern template class std::vector<float>;
extern template class std::vector<int>;
#endif // _MSC_VER

/**
  Assert that container is empty
  */
template <class T>
::testing::AssertionResult Empty(const std::vector< T > &container)
{
    if(container.empty()) { return ::testing::AssertionSuccess(); }
    else {
        return ::testing::AssertionFailure()
                << "Container is not empty and contains"
                << container.size() << " elements.";
    }
}
#define EXPECT_EMPTY(x) EXPECT_TRUE( Empty(x) )

#define EXPECT_SIZE(sz, container) EXPECT_EQ(size_t(sz), container.size())

#endif // _ELM_TS_CONTAINER_H_
