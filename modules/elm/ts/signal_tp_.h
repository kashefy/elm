/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** Setting up type-parameterized tests for Signal_ template class
 *  To subscribe to these tests for your dervied template of Signal_:
 *
 *  - Include this header.
 *  - Add to you unit test source file:
 *      typedef ::testing::Types<int, cv::Mat1f, ...> SignalFeatType;
 *      INSTANTIATE_TYPED_TEST_CASE_P(UniqueTestCollectionName, Signal_TP_, SignalFeatType);
 *      OR if it's only a single type, you can do:
 *      INSTANTIATE_TYPED_TEST_CASE_P(UniqueTestCollectionName, Signal_TP_, int);
 *
 *  - Initalize V_ with 3 values for each of your type,
 *    please check value usage in test case' SetUP() and fixtures below:
 *  - It is important to stay consistent with the logic in the tests that make use of these values.
 *    Example:
 *      template<> std::vector<int> V_<int>::values{0, 1, 100};
 *      template<> std::vector<cv::Mat1f > V_< cv::Mat1f >::values{Mat1f(1, 1, 0.f), Mat1f(1, 1, 1.f), Mat1f(1, 1, 100.f)};
 *
 *  for more details, please see: https://code.google.com/p/googletest/wiki/AdvancedGuide#Type-Parameterized_Tests
 */
#ifndef _ELM_TS_SIGNAL_TP__H_
#define _ELM_TS_SIGNAL_TP__H_

#include "gtest/gtest.h"

#include "elm/core/exception.h"
#include "elm/core/signal_.h"
#include "elm/ts/container.h"

// define some helper routines and classes

namespace elm {

/**
 * @brief the struct below, is a helper struct.
 * It enables defining values to be used inside the tests
 * These values are set below once per type.
 */
template<class T>
struct FtV_
{
    static std::vector<T> values;
};

/**
 * @brief A setup for repeating tests with different types of mat objects (int, float, uchar)
 */
template <class T>
class Signal_TP_ : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Signal_<T >();

        names_.clear();
        names_.push_back("foo");
        names_.push_back("bar");

        ASSERT_GE(FtV_<T >::values.size(), names_.size()) << "Not enough values provided for setting up fixtures.";
        ASSERT_GE(FtV_<T >::values.size(), size_t(3)) << "Need at least 3 values.";

        for(uint i=0; i<names_.size(); i++) {

            to_.Append(names_[i], FtV_<T >::values[i]);
        }

        to_.Append(names_[0], FtV_<T >::values[0]);
    }

    // members
    Signal_<T > to_;    ///< test object used in fixtures
    VecS names_;        ///< feature names
};

TYPED_TEST_CASE_P(Signal_TP_);

/**
 * @brief test utility function for populating a vector with uniformly random values
 */
TYPED_TEST_P(Signal_TP_, Constructor)
{
    EXPECT_NO_THROW(Signal_<TypeParam>());
}

TYPED_TEST_P(Signal_TP_, Clear)
{
    EXPECT_NO_THROW(Signal_<TypeParam>().Clear()) << "Redundant but should still work smoothly.";

    ASSERT_FALSE(this->to_.FeatureNames().empty()) << "Signal should contain something for this test.";
    EXPECT_NO_THROW(this->to_.Clear());

    EXPECT_TRUE(this->to_.FeatureNames().empty());

    EXPECT_NO_THROW(this->to_.Clear()) << "Again, redundant but should still work smoothly.";
}

TYPED_TEST_P(Signal_TP_, FeatureNames)
{
    EXPECT_EMPTY(Signal_<TypeParam>().FeatureNames()) << "Signal should initially be empty";

    VecS feature_names = this->to_.FeatureNames();
    EXPECT_EQ(size_t(2), feature_names.size());

    // linear search to see if feature names were added
    for(size_t i=0; i<this->names_.size(); i++) {

        bool found = false;
        for(size_t j=0; j<feature_names.size() && !found; j++) {

            found = feature_names[j] == this->names_[i];
        }
        EXPECT_TRUE(found) << "Could not find added name " << this->names_[i];
    }
}

TYPED_TEST_P(Signal_TP_, GetFeatures)
{
    EXPECT_THROW(this->to_.GetFeatureData("wrong"),   elm::ExceptionKeyError);
    EXPECT_THROW(this->to_.GetFeatureData("Foo"),     elm::ExceptionKeyError);
    EXPECT_THROW(this->to_.GetFeatureData("FOO"),     elm::ExceptionKeyError);

    EXPECT_SIZE(2, this->to_.GetFeatureData("foo"));
    EXPECT_SIZE(1, this->to_.GetFeatureData("bar"));

    //EXPECT_MAT_EQ(this->to_.GetFeatureData("bar")[0], );
}

TYPED_TEST_P(Signal_TP_, MostRecent_Invalid)
{
    EXPECT_THROW(this->to_.MostRecent("wrong"), elm::ExceptionKeyError);
    EXPECT_THROW(this->to_.MostRecent("Foo"), elm::ExceptionKeyError);
    EXPECT_THROW(this->to_.MostRecent("FOO"), elm::ExceptionKeyError);
}

TYPED_TEST_P(Signal_TP_, Append)
{
    ASSERT_GE(FtV_<TypeParam >::values.size(), size_t(3)) << "Need at least three value for this fixture.";

    const std::string FT_NAME = this->names_[0];
    const size_t SZ_INIT = this->to_.GetFeatureData(FT_NAME).size();

    ASSERT_GT(SZ_INIT, size_t(0)) << "SetUp should have added a feature under \"" << FT_NAME << "\"";

    for(int i=0; i<3; i++) {

        EXPECT_SIZE(SZ_INIT+i, this->to_.GetFeatureData(FT_NAME));

        this->to_.Append(FT_NAME, FtV_<TypeParam >::values[i%3]);

        EXPECT_SIZE(SZ_INIT+i+1, this->to_.GetFeatureData(FT_NAME)) << "Size of feature data did not increase as expected.";
    }
}

/** @brief This test looks at the most recently added feature.
 * This involves checking for equality
 * GTEST's EXPECT_EQ macro only works if the == operator is defined for the type being passed.
 * This is not the case for OpenCV Mat, which we make heavy use of
 * However, the << operator is defined for Mat as well as other types likely to be used.
 * So we'll try to cirvumvent this limitation by comparing the objects in the form of string comparisons
 *
 * We'll have to revisit this, when that assumption fails.
 */
TYPED_TEST_P(Signal_TP_, MostRecent_after_Append)
{
    ASSERT_GE(FtV_<TypeParam >::values.size(), size_t(3)) << "Need at least three value for this fixture.";

    const std::string FT_NAME = this->names_[0];
    const size_t SZ_INIT = this->to_.GetFeatureData(FT_NAME).size();

    ASSERT_GT(SZ_INIT, size_t(0)) << "SetUp should have added a feature under \"" << FT_NAME << "\"";

    for(int i=0; i<3; i++) {

        EXPECT_SIZE(SZ_INIT+i, this->to_.GetFeatureData(FT_NAME));

        this->to_.Append(FT_NAME, FtV_<TypeParam >::values[i%3]);

        EXPECT_SIZE(SZ_INIT+i+1, this->to_.GetFeatureData(FT_NAME)) << "Size of feature data did not increase as expected.";

        std::stringstream ss1, ss2;
        TypeParam most_recent = this->to_.MostRecent(FT_NAME);
        ss1 << most_recent;
        ss2 << FtV_<TypeParam >::values[i%3];
        EXPECT_GT(ss1.str().size(), size_t(0)) << "Unexpectedly empty.";
        EXPECT_EQ(ss1.str(), ss2.str()) << "Unexpected item found as most recent.";
    }
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(Signal_TP_,
                           Constructor,
                           Clear,
                           FeatureNames,
                           GetFeatures,
                           MostRecent_Invalid,
                           Append,
                           MostRecent_after_Append
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

} // namespace elm

#endif // _ELM_TS_SIGNAL_TP__H_
