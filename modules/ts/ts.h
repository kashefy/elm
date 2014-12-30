/** Include this header to include other headers with
 * - custom assertions
 * - test-related routines
 * Don't forget to add new headers as you create them
  */
#ifndef SEM_TS_H_
#define SEM_TS_H_

#include "gtest/gtest.h"

#include <string>

#include "ts/interval.h"
#include "ts/mat_assertions.h"
#include "ts/container.h"

const char* FullTestName(const ::testing::TestInfo *test_info);

#endif // SEM_TS_H_
