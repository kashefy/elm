/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/concatentatecloudxyzandnormal.h"

#ifdef __WITH_PCL // test normally

#include <pcl/point_types.h>

#include "elm/core/featuredata.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/core/pcl/point_traits.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/mat_assertions.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

extern template class pcl::PointCloud<pcl::PointXYZ >;
extern template class pcl::PointCloud<pcl::Normal >;
extern template class pcl::PointCloud<pcl::PointNormal >;

extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::Normal > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > >;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ConcatentateCloudXYZAndNormal);

// Names for I/O
const string NAME_INPUT_XYZ     = "xyz";        ///< name of input point cloud
const string NAME_INPUT_NORMAL  = "normal";     ///< name of input point cloud
const string NAME_OUTPUT        = "out";             ///< name of output vertices

class ConcatentateCloudXYZAndNormalInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cfg_ = LayerConfig();
        cfg_.Params(params_);

        io_names_.Input(ConcatentateCloudXYZAndNormal::KEY_INPUT_XYZ, NAME_INPUT_XYZ);
        io_names_.Input(ConcatentateCloudXYZAndNormal::KEY_INPUT_NORMAL, NAME_INPUT_NORMAL);
        io_names_.Output(ConcatentateCloudXYZAndNormal::KEY_OUTPUT_POINT_NORMAL, NAME_OUTPUT);
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

TEST_F(ConcatentateCloudXYZAndNormalInitTest, MissingParams)
{
    cfg_.Params(PTree());

    ConcatentateCloudXYZAndNormal to;
    EXPECT_NO_THROW(to.Reset(cfg_));
    EXPECT_NO_THROW(to.Reconfigure(cfg_));
}

TEST_F(ConcatentateCloudXYZAndNormalInitTest, CreateWithFactory)
{
    LayerShared to_ptr = LayerFactory::CreateShared("ConcatentateCloudXYZAndNormal", cfg_, io_names_);

    EXPECT_TRUE(bool(to_ptr));
}

Normal initNormal(float nx, float ny, float nz, float c) {

    Normal p(nx, ny, nz);
    p.curvature = c;
    return p;
}

/**
 * @brief Test the live methods assuming a sucessful initialization
 */
class ConcatentateCloudXYZAndNormalTest : public ConcatentateCloudXYZAndNormalInitTest
{
protected:
    virtual void SetUp()
    {
        ConcatentateCloudXYZAndNormalInitTest::SetUp();

        // Load input file into a PointCloudXYZ
        cld_xyz_.reset(new CloudXYZ);

        cld_xyz_->push_back(PointXYZ( 1.f,  2.f,  3.f));
        cld_xyz_->push_back(PointXYZ(11.f, 12.f, 13.f));
        cld_xyz_->push_back(PointXYZ(21.f, 22.f, 23.f));
        cld_xyz_->push_back(PointXYZ(31.f, 32.f, 33.f));
        cld_xyz_->push_back(PointXYZ(41.f, 42.f, 43.f));
        cld_xyz_->push_back(PointXYZ(51.f, 52.f, 53.f));
        cld_xyz_->push_back(PointXYZ(61.f, 62.f, 63.f));

        cld_nrml_.reset(new CloudNormal);

        cld_nrml_->push_back(initNormal(101.f, 102.f, 103.f, 104.f));
        cld_nrml_->push_back(initNormal(111.f, 112.f, 113.f, 114.f));
        cld_nrml_->push_back(initNormal(121.f, 122.f, 123.f, 124.f));
        cld_nrml_->push_back(initNormal(131.f, 132.f, 133.f, 134.f));
        cld_nrml_->push_back(initNormal(141.f, 142.f, 143.f, 144.f));
        cld_nrml_->push_back(initNormal(151.f, 152.f, 153.f, 154.f));
        cld_nrml_->push_back(initNormal(161.f, 162.f, 163.f, 164.f));

        to_ = LayerFactory::CreateShared("ConcatentateCloudXYZAndNormal", cfg_, io_names_);

        sig_.Append(NAME_INPUT_XYZ, cld_xyz_);

        sig_.Append(NAME_INPUT_NORMAL, FeatureData(cld_nrml_));
    }

    virtual void TearDown()
    {
        ConcatentateCloudXYZAndNormalInitTest::TearDown();
        sig_.Clear();
    }

    LayerShared to_; ///< pointer to test object
    Signal sig_;
    CloudXYZPtr cld_xyz_;
    CloudNrmlPtr cld_nrml_;
};

TEST_F(ConcatentateCloudXYZAndNormalTest, ActivateEmptyInput)
{
    sig_.Clear();

    cld_xyz_.reset(new CloudXYZ);
    cld_nrml_.reset(new CloudNormal);

    sig_.Append(NAME_INPUT_XYZ, cld_xyz_);
    sig_.Append(NAME_INPUT_NORMAL, cld_nrml_);

    ASSERT_FALSE(sig_.Exists(NAME_OUTPUT));

    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT));

    CloudPtNrmlPtr out = sig_.MostRecent(NAME_OUTPUT).get<CloudPtNrmlPtr>();

    EXPECT_TRUE(out->empty());
}

TEST_F(ConcatentateCloudXYZAndNormalTest, ActivateAndResponse)
{
    ASSERT_FALSE(sig_.Exists(NAME_OUTPUT));

    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT)) << "concatenated point cloud is missing";

    CloudPtNrmlPtr out = sig_.MostRecent(NAME_OUTPUT).get<CloudPtNrmlPtr>();

    EXPECT_EQ(cld_xyz_->width, out->width);
    EXPECT_EQ(cld_xyz_->height, out->height);
    EXPECT_EQ(cld_xyz_->size(), out->size());

    EXPECT_EQ(cld_nrml_->width, out->width);
    EXPECT_EQ(cld_nrml_->height, out->height);
    EXPECT_EQ(cld_nrml_->size(), out->size());

    PointCloud<PointXYZ>::const_iterator itr1 = cld_xyz_->begin();
    PointCloud<Normal>::const_iterator itr2 = cld_nrml_->begin();
    PointCloud<PointNormal>::const_iterator itr3 = out->begin();
    for(int i=0; itr1 != cld_xyz_->end(); i++, ++itr1, ++itr2, ++itr3) {

        PointXYZ _p1 = *itr1;
        Normal _p2 = *itr2;
        PointNormal _p3 = *itr3;

        for(int k=0; k<3; k++) {

            // xyz components
            EXPECT_FLOAT_EQ(_p1.data[k], _p3.data[k]) << "xyz component k=" << k << "mismatch at element i=" << i;

            // normal components
            EXPECT_FLOAT_EQ(_p2.normal[k], _p3.normal[k]) << "normal component k=" << k << "mismatch at element i=" << i;

            EXPECT_FLOAT_EQ(_p2.curvature, _p3.curvature) << "curvature mismatch at element i=" << i;

        }
    }
}

} // annonymous namespace for test fixtures

#endif // __WITH_PCL

