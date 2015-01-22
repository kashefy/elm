/** Include this header to include other headers with
 * - custom assertions
 * - test-related routines
 * Don't forget to add new headers as you create them
  */
#ifndef ELM_TS_H_
#define ELM_TS_H_

#include "gtest/gtest.h"

#include <string>

#include "sem/ts/interval.h"
#include "sem/ts/mat_assertions.h"
#include "sem/ts/container.h"

namespace elm
{

const char* FullTestName(const ::testing::TestInfo *test_info);

} // namespace elm

#endif // ELM_TS_H_
