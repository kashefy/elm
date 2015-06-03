/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/pointnormalestimation.h"

#ifdef __WITH_PCL // test normally

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include "elm/core/exception.h"
#include "elm/core/featuredata.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace pcl;
using namespace elm;

extern template class pcl::PointCloud<pcl::PointXYZ >;
extern template class pcl::PointCloud<pcl::Normal >;

extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::Normal > >;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(PointNormalEstimation);

const bfs::path TEST_DIR("testdata");
const bfs::path TEST_PATH_PCD = TEST_DIR/"bun0.pcd";

// Names for I/O
const string NAME_INPUT_POINT_CLOUD     = "in";     ///< name of input point cloud
const string NAME_OUTPUT_POINT_CLOUD    = "out";    ///< name of output cloud with nornals

class PointNormalEstimationInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cfg_ = LayerConfig();
        params_.clear();
        params_.put<int>(PointNormalEstimation::PARAM_K_SEARCH, 20);
        cfg_.Params(params_);

        io_names_.Input(PointNormalEstimation::KEY_INPUT_STIMULUS, NAME_INPUT_POINT_CLOUD);
        io_names_.Output(PointNormalEstimation::KEY_OUTPUT_POINT_CLOUD, NAME_OUTPUT_POINT_CLOUD);
    }

    virtual void TearDown()
    {
        params_.clear();
        io_names_ = LayerIONames();
    }

    // members
    PTree params_;
    LayerConfig cfg_;
    LayerIONames io_names_;
};

TEST_F(PointNormalEstimationInitTest, MissingParams)
{
    cfg_.Params(PTree());

    PointNormalEstimation to;
    EXPECT_THROW(to.Reset(cfg_), boost::property_tree::ptree_bad_path);
    EXPECT_THROW(to.Reconfigure(cfg_), boost::property_tree::ptree_bad_path);
}

TEST_F(PointNormalEstimationInitTest, CreateWithFactory)
{
    LayerShared ptr = LayerFactory::CreateShared("PointNormalEstimation",
                                                 cfg_,
                                                 io_names_);

    EXPECT_TRUE(bool(ptr));
}

/**
 * @brief Test the live methods assuming a sucessful initialization
 */
class PointNormalEstimationTest : public PointNormalEstimationInitTest
{
protected:
    virtual void SetUp()
    {
        PointNormalEstimationInitTest::SetUp();

        // Load input file into a PointCloudXYZ
        cloud_in_.reset(new CloudXYZ);
        PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(TEST_PATH_PCD.string(), cloud_blob);
        fromPCLPointCloud2 (cloud_blob, *cloud_in_);

        to_ = LayerFactory::CreateShared("PointNormalEstimation", cfg_, io_names_);

        sig_.Append(NAME_INPUT_POINT_CLOUD, cloud_in_);
    }

    virtual void TearDown()
    {
        PointNormalEstimationInitTest::TearDown();
        sig_.Clear();
    }

    LayerShared to_; ///< pointer to test object
    Signal sig_;
    CloudXYZPtr cloud_in_;
};

TEST_F(PointNormalEstimationTest, ActivateEmptyInput)
{
    cloud_in_->clear();
    sig_.Append(NAME_INPUT_POINT_CLOUD, cloud_in_);
    EXPECT_THROW(to_->Activate(sig_), ExceptionBadDims);
}

TEST_F(PointNormalEstimationTest, ActivateAndResponse)
{
    // Normal estimation
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

    {
        NormalEstimation<PointXYZ, Normal> n;
        search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

        tree->setInputCloud(cloud_in_);
        n.setInputCloud(cloud_in_);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);
        // normals should now contain the point normals + surface curvatures

        ASSERT_GT(normals->size(), static_cast<size_t>(0));
    }

    to_->Activate(sig_);

    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_POINT_CLOUD)) << "Output feature already exists, better clear signal first.";

    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_POINT_CLOUD)) << "Output feature is missing.";

    CloudNrmlPtr response = sig_.MostRecent(NAME_OUTPUT_POINT_CLOUD).get<CloudNrmlPtr>();
    EXPECT_EQ(normals->width, response->width);
    EXPECT_EQ(normals->height, response->height);
    EXPECT_EQ(normals->size(), response->size());

    PointCloud<Normal>::const_iterator itr1 = normals->begin();
    PointCloud<Normal>::const_iterator itr2 = response->begin();
    for(int i=0; itr1 != normals->end(); i++, ++itr1, ++itr2) {

        Normal _p1 = *itr1;
        Normal _p2 = *itr2;

        EXPECT_NE(_p1.normal, _p2.normal);

        for(int k=0; k<3; k++) {

            EXPECT_FLOAT_EQ(_p1.normal[k], _p2.normal[k]) << "normal component k=" << k << "mismatch at element i=" << i;
        }
    }
}

TEST_F(PointNormalEstimationTest, ResponseDims)
{
    for(uint32_t r=30; r<46; r+=3) {

        for(uint32_t c=30; c<46; c+=3) {

            sig_.Clear();

            cloud_in_.reset(new CloudXYZ(c, r));
            sig_.Append(NAME_INPUT_POINT_CLOUD, cloud_in_);

            to_->Activate(sig_);
            to_->Response(sig_);

            CloudNrmlPtr response = sig_.MostRecent(NAME_OUTPUT_POINT_CLOUD).get<CloudNrmlPtr>();
            EXPECT_EQ(c, response->width);
            EXPECT_EQ(r, response->height);
            EXPECT_EQ(static_cast<size_t>(r*c), response->size());
        }
    }
}

} // annonymous namespace for test fixtures

#endif // __WITH_PCL

