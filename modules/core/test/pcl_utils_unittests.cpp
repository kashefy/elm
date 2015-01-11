#include "core/pcl_utils.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

TEST(PCLUtilsMat2PointCloudTEST, Empty)
{
    EXPECT_TRUE(Mat2PointCloud(Mat1f())->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat1f(1, 0))->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat1f(0, 1))->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat1f(0, 0))->empty());
}

TEST(PCLUtilsMat2PointCloudTEST, Empty3Ch)
{
    EXPECT_TRUE(Mat2PointCloud(Mat3f())->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat3f(1, 0))->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat3f(0, 1))->empty());
    EXPECT_TRUE(Mat2PointCloud(Mat3f(0, 0))->empty());
}

TEST(PCLUtilsMat2PointCloudTEST, EmptySizeZero)
{
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat1f())->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat1f(1, 0))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat1f(0, 1))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat1f(0, 0))->size());
}

TEST(PCLUtilsMat2PointCloudTEST, EmptySizeZero3Ch)
{
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat3f())->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat3f(1, 0))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat3f(0, 1))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud(Mat3f(0, 0))->size());
}

TEST(PCLUtilsMat2PointCloudTEST, Invalid)
{
    EXPECT_THROW(Mat2PointCloud(Mat1f::ones(1, 1)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud(Mat1f::ones(2, 1)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud(Mat1f::ones(1, 2)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud(Mat1f::ones(2, 2)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud(Mat1f::ones(4, 1)), ExceptionBadDims);
}

TEST(PCLUtilsMat2PointCloudTEST, Dims)
{
    Mat1f m(1, 9);
    randn(m, 0.f, 100.f);

    CloudXYZ::Ptr cloud_ptr;

    cloud_ptr = Mat2PointCloud(m);
    EXPECT_EQ(m.total()/3, cloud_ptr->size());
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));

    cloud_ptr = Mat2PointCloud(m.reshape(1, 3));
    EXPECT_EQ(m.total()/3, cloud_ptr->size());
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->height));

    Mat1f m2(4, 3);
    randn(m2, 0.f, 100.f);
    cloud_ptr = Mat2PointCloud(m2);
    EXPECT_EQ(4, static_cast<int>(cloud_ptr->size()));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(4, static_cast<int>(cloud_ptr->height));

    EXPECT_NO_THROW(Mat2PointCloud(Mat1f::zeros(3, 4)));
    cloud_ptr = Mat2PointCloud(Mat1f::zeros(3, 4));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->size()));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->height));
}

TEST(PCLUtilsMat2PointCloudTEST, Dims3Ch)
{
    Mat3f m(1, 9);
    randn(m, 0.f, 100.f);

    CloudXYZ::Ptr cloud_ptr;

    cloud_ptr = Mat2PointCloud(m);
    EXPECT_EQ(m.total(), cloud_ptr->size());
    EXPECT_EQ(9, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));

    cloud_ptr = Mat2PointCloud(m.reshape(1, 3));
    EXPECT_EQ(m.total(), cloud_ptr->size());
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->height));

    Mat3f m2(4, 3);
    randn(m2, 0.f, 100.f);
    cloud_ptr = Mat2PointCloud(m2);
    EXPECT_EQ(4*3, static_cast<int>(cloud_ptr->size()));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(4, static_cast<int>(cloud_ptr->height));
}

TEST(PCLUtilsMat2PointCloudTEST, IsOrganized)
{
    Mat1f m(9, 9);
    randn(m, 0.f, 100.f);

    EXPECT_TRUE(Mat2PointCloud(m)->isOrganized());
    EXPECT_TRUE(Mat2PointCloud(m.reshape(1, 3))->isOrganized());
    EXPECT_FALSE(Mat2PointCloud(m.reshape(1, 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

TEST(PCLUtilsMat2PointCloudTEST, IsOrganized3Ch)
{
    Mat3f m(9, 9);
    randn(m, 0.f, 100.f);

    EXPECT_TRUE(Mat2PointCloud(m)->isOrganized());
    EXPECT_TRUE(Mat2PointCloud(m.reshape(1, 3))->isOrganized());
    EXPECT_FALSE(Mat2PointCloud(m.reshape(1, 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

/**
 * @brief check values stored in non-organized point cloud after conversion
 */
TEST(PCLUtilsMat2PointCloudTEST, ValuesNonOrganized)
{
    Mat1f m(1, 9);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    int i=0;
    for(CloudXYZ::iterator itr=cloud_ptr->begin(); itr != cloud_ptr->end(); ++itr, i+=3) {

        PointXYZ _p = *itr;
        EXPECT_FLOAT_EQ(m(i)  , _p.x) << "Unexpected value for x coordinate.";
        EXPECT_FLOAT_EQ(m(i+1), _p.y) << "Unexpected value for y coordinate.";
        EXPECT_FLOAT_EQ(m(i+2), _p.z) << "Unexpected value for z coordinate.";
    }
}

TEST(PCLUtilsMat2PointCloudTEST, ValuesNonOrganized3Ch)
{
    Mat3f m(1, 9);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    int i=0;
    for(CloudXYZ::iterator itr=cloud_ptr->begin(); itr != cloud_ptr->end(); ++itr, i++) {

        PointXYZ _p = *itr;
        Vec3f _pm = m(i);
        EXPECT_FLOAT_EQ(_pm[0], _p.x) << "Unexpected value for x coordinate.";
        EXPECT_FLOAT_EQ(_pm[1], _p.y) << "Unexpected value for y coordinate.";
        EXPECT_FLOAT_EQ(_pm[2], _p.z) << "Unexpected value for z coordinate.";
    }
}

/**
 * @brief check values stored in organized point cloud after conversion
 */
TEST(PCLUtilsMat2PointCloudTEST, ValuesOrganized)
{
    // organized point cloud
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr->width; c++, i+=3) {

            PointXYZ _p = cloud_ptr->at(c, r);
            EXPECT_FLOAT_EQ(m(r, i)  , _p.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(m(r, i+1), _p.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(m(r, i+2), _p.z) << "Unexpected value for z coordinate.";
        }
    }
}

TEST(PCLUtilsMat2PointCloudTEST, ValuesOrganized3Ch)
{
    // organized point cloud
    Mat3f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr->width; c++, i++) {

            PointXYZ _p = cloud_ptr->at(c, r);
            Vec3f _pm = m(r, c);
            EXPECT_FLOAT_EQ(_pm[0], _p.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(_pm[1], _p.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(_pm[2], _p.z) << "Unexpected value for z coordinate.";
        }
    }
}

/**
 * @brief Populate a single and 3-channel matrix with same values
 * and compare that both result in the same point cloud.
 */
TEST(PCLUtilsMat2PointCloudTEST, SingleChannelVs3ChannelMat)
{
    // organized point cloud
    Mat1f m1Ch(3, 6);
    randn(m1Ch, 0.f, 100.f);

    Mat3f m3Ch(3, 2);
    for(size_t i=0, j=0; i<m1Ch.total(); i+=3) {
        Vec3f tmp;
        tmp[0] = m1Ch(i);
        tmp[1] = m1Ch(i+1);
        tmp[2] = m1Ch(i+2);
        m3Ch(j++) = tmp;
    }

    CloudXYZ::Ptr cloud_ptr1Ch = Mat2PointCloud(m1Ch);
    CloudXYZ::Ptr cloud_ptr3Ch = Mat2PointCloud(m3Ch);

    for(size_t r=0; r<cloud_ptr1Ch->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr1Ch->width; c++, i++) {

            PointXYZ _p1 = cloud_ptr1Ch->at(c, r);
            PointXYZ _p2 = cloud_ptr3Ch->at(c, r);
            EXPECT_FLOAT_EQ(_p2.x, _p1.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(_p2.y, _p1.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(_p2.z, _p1.z) << "Unexpected value for z coordinate.";
        }
    }
}

TEST(PCLUtilsPointCloud2MatTEST, Empty)
{
    CloudXYZ::Ptr cloud_ptr;
    cloud_ptr.reset(new CloudXYZ());
    EXPECT_TRUE(PointCloud2Mat(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(0, 0));
    EXPECT_TRUE(PointCloud2Mat(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(0, 1));
    EXPECT_TRUE(PointCloud2Mat(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(1, 0));
    EXPECT_TRUE(PointCloud2Mat(cloud_ptr).empty());
}


TEST(PCLUtilsPointCloud2MatTEST, Dims)
{
    Mat1f m(9, 9);
    randn(m, 0.f, 100.f);

    CloudXYZ::Ptr cloud_ptr;

    cloud_ptr = Mat2PointCloud(m);
    EXPECT_MAT_DIMS_EQ(PointCloud2Mat(cloud_ptr), Size2i(12, m.rows));

    cloud_ptr = Mat2PointCloud(m.reshape(1, 27));
    EXPECT_MAT_DIMS_EQ(PointCloud2Mat(cloud_ptr), Size2i(4, 27));
}

TEST(PCLUtilsPointCloud2MatTEST, Values)
{
    // organized point cloud
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    Mat1f m2 = PointCloud2Mat(cloud_ptr);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
}

TEST(PCLUtilsPointCloud2MatTEST, Owner)
{
    // organized point cloud
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud_ptr = Mat2PointCloud(m);

    Mat1f m2 = PointCloud2Mat(cloud_ptr);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr->width; c++, i+=4) {

            PointXYZ _p = cloud_ptr->at(c, r);
            EXPECT_FLOAT_EQ(m2(r, i)  , _p.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+1), _p.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+2), _p.z) << "Unexpected value for z coordinate.";

            PointXYZ _p2;
            _p2.x += 1.f;
            _p2.y += 2.f;
            _p2.z += 3.f;

            cloud_ptr->at(c, r) = _p2;

            EXPECT_FLOAT_EQ(m2(r, i)  , _p2.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+1), _p2.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+2), _p2.z) << "Unexpected value for z coordinate.";
        }
    }
}

TEST(PCLUtilsMat2VecVerticesTest, Empty)
{
    EXPECT_TRUE(Mat2VecVertices(Mat_f()).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(1, 0)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(0, 1)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(0, 0)).empty());
    EXPECT_TRUE(Mat2VecVertices(Mat_f(3, 0)).empty());
}

TEST(PCLUtilsMat2VecVerticesTest, Non_continuous)
{
    Mat1f m(3, 4, 1.f);
    EXPECT_NO_THROW(Mat2VecVertices(m.colRange(0, 2)));
    EXPECT_NO_THROW(Mat2VecVertices(m.col(0)));
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
                    vector<Vertices > vv = Mat2VecVertices(m);

                    for(size_t i=0; i<vv.size(); i++) {

                        EXPECT_SIZE(size_t((ch==1)? cols : ch), vv[i].vertices) << "Unexpected length of Vertices.";
                    }
                }
            }
        }
    }
}

/**
 * @brief test vertex values after conversion from a single-channel Mat of floats
 * Randomize the matrix with positive values since, Verticie stores them as uint32_t
 */
TEST(PCLUtilsMat2VecVerticesTest, VerticesValues_single_ch)
{
    for(int cols=1; cols<5; cols++) {

        Mat1f m(3, cols);
        // fill with values that make sense when represented as uint32_t
        for(size_t i=0; i<m.total(); i++) {

            m(i) = static_cast<float>(randu<uint32_t>() % 256);
        }
        vector<Vertices > vv = Mat2VecVertices(m);

        for(size_t i=0; i<vv.size(); i++) {

            Vertices v = vv[i];
            for(size_t j=0; j<v.vertices.size(); j++) {

                float vertex = static_cast<float>(v.vertices[j]);
                EXPECT_FLOAT_EQ(m(i, j), vertex) << "Vertex mismatch at " << i << "," << j;
            }
        }
    }
}

/**
 * @brief test vertex values after conversion from 3-channel Mat of floats
 * Randomize the matrix with positive values since, Verticie stores them as uint32_t
 */
TEST(PCLUtilsMat2VecVerticesTest, VerticesValues_ch3)
{
    Mat3f m(3, 1);

    // fill with values that make sense when represented as uint32_t
    for(size_t i=0; i<m.total(); i++) {

        for(int ch=0; ch<m.channels(); ch++) {

            m(i)[ch] = static_cast<float>(randu<uint32_t>() % 256);
        }
    }

    vector<Vertices > vv = Mat2VecVertices(m);

    for(size_t i=0; i<vv.size(); i++) {

        Vertices v = vv[i];
        for(size_t j=0; j<v.vertices.size(); j++) {

            float vertex = static_cast<float>(v.vertices[j]);
            EXPECT_FLOAT_EQ(m(i)[j], vertex) << "Vertex mismatch at " << i << "," << j;
        }
    }
}
