/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphmap.h"

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils_inl.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace elm;

namespace {

class GraphMapConstructTest : public ::testing::Test
{
protected:
};

} // annonymous namespace for test cases and fixtures
