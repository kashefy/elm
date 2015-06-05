/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** Include this header to include other headers with
 * - custom assertions
 * - test-related routines
 * Don't forget to add new headers as you create them
  */
#ifndef _ELM_TS_H_
#define _ELM_TS_H_

#include "gtest/gtest.h"

#include "elm/ts/interval.h"
#include "elm/ts/mat_assertions.h"
#include "elm/ts/container.h"

namespace elm
{

const char* FullTestName(const ::testing::TestInfo *test_info);

} // namespace elm

#endif // _ELM_TS_H_
