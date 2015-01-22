/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/vertices.h"

#ifdef __WITH_PCL // PCL support required for these tests

#include "elm/core/exception.h"
#include "elm/core/typedefs_fwd.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

// tests around Mat <-> VecVertices conversion, with fixtures for single-channel Mat, and 3-channel Mat

TEST(PCLUtilsMat2VecVerticesTest, Empty)
{
    EXPECT_TRUE(Mat2VecVertices(Mat_f()).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(1, 0)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(0, 1)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(0, 0)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(3, 0)).empty());

    for(int ch=1; ch<4; ch++) {

        EXPECT_TRUE(Mat2VecVertices(Mat(0, 0, CV_MAKETYPE(CV_32F, ch))).empty());
        EXPECT_TRUE(Mat2VecVertices(Mat(0, 1, CV_MAKETYPE(CV_32F, ch))).empty());
        EXPECT_TRUE(Mat2VecVertices(Mat(1, 0, CV_MAKETYPE(CV_32F, ch))).empty());
        EXPECT_TRUE(Mat2VecVertices(Mat(0, 3, CV_MAKETYPE(CV_32F, ch))).empty());
        EXPECT_TRUE(Mat2VecVertices(Mat(5, 0, CV_MAKETYPE(CV_32F, ch))).empty());
    }
}

/**
 * @brief check size of Vertices vector after conversion, expecting it to match no. of rows in input Mat
 */
TEST(PCLUtilsMat2VecVerticesTest, Nb_Verticies)
{
    for(int r=1; r<11; r++) {

        for(int ch=1; ch<4; ch++) {

            for(int cols=1; cols<5; cols++) {

                if(ch > 1 && cols > 1) {

                    continue;
                }
                else {

                    Mat m(r, cols, CV_MAKETYPE(CV_32F, ch), Scalar_<float>(1.f));
                    EXPECT_SIZE(size_t(r), Mat2VecVertices(m)) << "Unexpected no. of Vertices.";
                }
            }
        }
    }
}

TEST(PCLUtilsMat2VecVerticesTest, Invalid_multi_ch_multi_col)
{
    for(int ch=1; ch<4; ch++) {

        for(int cols=1; cols<5; cols++) {

            Mat m(3, cols, CV_MAKETYPE(CV_32F, ch), Scalar_<float>(1.f));
            if(ch > 1 && m.cols > 1) {

                EXPECT_THROW(Mat2VecVertices(m), ExceptionBadDims);
            }
            else {

                EXPECT_NO_THROW(Mat2VecVertices(m));
            }
        }
    }
}

/**
 * @brief test length of each Vertices object in vector after conversion
 */
TEST(PCLUtilsMat2VecVerticesTest, Len_Verticies)
{
    for(int r=1; r<3; r++) {

        for(int ch=1; ch<4; ch++) {

            for(int cols=1; cols<5; cols++) {

                if(ch > 1 && cols > 1) {

                    continue;
                }
                else {

                    Mat m(r, cols, CV_MAKETYPE(CV_32F, ch), Scalar_<float>(1.f));
                    VecVertices vv = Mat2VecVertices(m);

                    for(size_t i=0; i<vv.size(); i++) {

                        EXPECT_SIZE(size_t((ch==1)? cols : ch), vv[i].vertices) << "Unexpected length of Vertices.";
                    }
                }
            }
        }
    }
}

class PCLUtilsMat2VecVerticesSingleChTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        m1ch_ = Mat1f(3, 4);

        // Randomize the matrix with positive values since, Verticie stores them as uint32_t
        // fill with values that make sense when represented as uint32_t
        for(size_t i=0; i<m1ch_.total(); i++) {

            m1ch_(i) = static_cast<float>(randu<uint32_t>() % 256);
        }
    }

    // members
    Mat1f m1ch_;
};

TEST_F(PCLUtilsMat2VecVerticesSingleChTest, Non_continuous)
{
    EXPECT_NO_THROW(Mat2VecVertices(m1ch_.colRange(0, 2)));
    EXPECT_NO_THROW(Mat2VecVertices(m1ch_.col(0)));
}

/**
 * @brief test vertex values after conversion from a single-channel Mat of floats
 * Randomize the matrix with positive values since, Verticie stores them as uint32_t
 */
TEST_F(PCLUtilsMat2VecVerticesSingleChTest, VerticesValues)
{
    for(int cols=1; cols<5; cols++) {

        Mat1f m(3, cols);
        // fill with values that make sense when represented as uint32_t
        for(size_t i=0; i<m.total(); i++) {

            m(i) = static_cast<float>(randu<uint32_t>() % 256);
        }
        VecVertices vv = Mat2VecVertices(m);

        for(size_t i=0; i<vv.size(); i++) {

            Vertices v = vv[i];
            for(size_t j=0; j<v.vertices.size(); j++) {

                float vertex = static_cast<float>(v.vertices[j]);
                EXPECT_FLOAT_EQ(m(i, j), vertex) << "Vertex mismatch at " << i << "," << j;
            }
        }
    }
}

TEST_F(PCLUtilsMat2VecVerticesSingleChTest, RoundTrip)
{
    VecVertices vv = Mat2VecVertices(m1ch_);
    {
        Mat1f m2 = VecVertices2Mat(vv, false);
        EXPECT_MAT_DIMS_EQ(m2, m1ch_);
    }
    {
        Mat1f m2 = VecVertices2Mat(vv, true);
        EXPECT_MAT_DIMS_EQ(m2.reshape(1, m1ch_.rows), m1ch_);
    }
}

class PCLUtilsMat2VecVertices3ChTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        m3ch_ = Mat3f(3, 1);

        // Randomize the matrix with positive values since, Verticie stores them as uint32_t
        // fill with values that make sense when represented as uint32_t
        for(size_t i=0; i<m3ch_.total(); i++) {

            m3ch_(i) = static_cast<float>(randu<uint32_t>() % 256);
        }
    }

    // members
    Mat3f m3ch_;
};

/**
 * @brief test vertex values after conversion from 3-channel Mat of floats
 * Randomize the matrix with positive values since, Verticie stores them as uint32_t
 */
TEST_F(PCLUtilsMat2VecVertices3ChTest, VerticesValues)
{
    VecVertices vv = Mat2VecVertices(m3ch_);

    for(size_t i=0; i<vv.size(); i++) {

        Vertices v = vv[i];
        for(size_t j=0; j<v.vertices.size(); j++) {

            float vertex = static_cast<float>(v.vertices[j]);
            EXPECT_FLOAT_EQ(m3ch_(i)[j], vertex) << "Vertex mismatch at " << i << "," << j;
        }
    }
}

TEST_F(PCLUtilsMat2VecVertices3ChTest, RoundTrip)
{
    VecVertices vv = Mat2VecVertices(m3ch_);
    Mat1f m2 = VecVertices2Mat(vv, false);

    EXPECT_MAT_DIMS_EQ(m2, Size2i(m3ch_.channels(), m3ch_.rows));

    for(int r=0; r<m2.rows; r++) {

        for(int c=0; c<m2.cols; c++) {

            EXPECT_FLOAT_EQ(m2(r, c), m3ch_(r)[c]) << "Vertex mismatch at " << r << ", " << c;
        }
    }
}

} // annonymous namespace around tests

#else // __WITH_PCL
    #warning "Skipping building pcl utils unit tests due to no pcl support."
#endif // __WITH_PCL
