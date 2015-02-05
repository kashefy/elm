#include "elm/layers/gradassignment.h"
#include "elm/layers/triangulation.h"

#ifdef __WITH_PCL

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include "elm/core/debug_utils.h"

#include "elm/core/exception.h"
#include "elm/core/graph/adjacency.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/ts.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

const bfs::path TEST_DIR("testdata");
const bfs::path TEST_PATH_PCD = TEST_DIR/"bun0.pcd";

// Names for Triangulation I/O
const string NAME_INPUT_POINT_CLOUD = "in";   ///< name of input point cloud
const string NAME_OUTPUT_VERTICES   = "v";    ///< name of output vertices

// Grad. assignment parameters
const float BETA            = 0.5f;
const float BETA_MAX        = 10.f;
const float BETA_RATE       = 1.075f;
const int MAX_ITER_PER_BETA = 4;
const int MAX_ITER_SINKHORN = 30;

// Names for Grad. assignment I/O
const string NAME_GRAPH_AB = "G_ab";
const string NAME_GRAPH_IJ = "g_ij";
const string NAME_GRAPH_COMPAT = "c_ai";
const string NAME_M        = "m_ga";

/**
 * @brief Test graph matching feature(s)
 */
class GraphMatchingTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        // set up layers
        {
            LayerConfig cfg;

            PTree p;

            cfg.Params(p);

            LayerIONames io;
            io.Input(Triangulation::KEY_INPUT_POINT_CLOUD,  NAME_INPUT_POINT_CLOUD);
            io.Output(Triangulation::KEY_OUTPUT_VERTICES,   NAME_OUTPUT_VERTICES);

            layers_.push_back(LayerFactory::CreateShared("Triangulation", cfg, io));
        }

        {
            LayerConfig cfg;

            PTree p;
            p.put(GradAssignment::PARAM_BETA, BETA);
            p.put(GradAssignment::PARAM_BETA_MAX, BETA_MAX);
            p.put(GradAssignment::PARAM_BETA_RATE, BETA_RATE);
            p.put(GradAssignment::PARAM_MAX_ITER_PER_BETA, MAX_ITER_PER_BETA);
            p.put(GradAssignment::PARAM_MAX_ITER_SINKHORN, MAX_ITER_SINKHORN);

            cfg.Params(p);

            LayerIONames io;
            io.Input(GradAssignment::KEY_INPUT_GRAPH_AB, NAME_GRAPH_AB);
            io.Input(GradAssignment::KEY_INPUT_GRAPH_IJ, NAME_GRAPH_IJ);
            io.Input(GradAssignment::KEY_INPUT_MAT_COMPATIBILITY, NAME_GRAPH_COMPAT);
            io.Output(GradAssignment::KEY_OUTPUT_RESPONSE, NAME_M);

            layers_.push_back(LayerFactory::CreateShared("GradAssignment", cfg, io));
        }

        // load data
        CloudXYZPtr cloud_in(new CloudXYZ);
        PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(TEST_PATH_PCD.string(), cloud_blob);
        fromPCLPointCloud2(cloud_blob, *cloud_in);

        /* CAUTION: When appending point cloud to signal, verify ownership is transfered also.
         *
         * Scenario a) where ownership is not transfered and the signal data becomes invalid:
         *
         * sig_.Append(NAME_INPUT_POINT_CLOUD, PointCloud2Mat_(cloud_in));
         *
         * Scenario b) where owenership is transfered through explicit deep copy AND increment of reference count (of Mat object):
         * sig_.Append(NAME_INPUT_POINT_CLOUD, PointCloud2Mat_(cloud_in));
         *
         * Scenario c) where data is preserved through increment of reference count (of cloud object): (used below)
         * sig_.Append(NAME_INPUT_POINT_CLOUD, cloud_in);
         */
        sig_.Append(NAME_INPUT_POINT_CLOUD, cloud_in);

    }

    virtual void TearDown()
    {
        layers_.clear();
        sig_.Clear();
    }

    // members
    vector<LayerFactory::LayerShared > layers_;
    Signal sig_;
};

TEST_F(GraphMatchingTest, GraphMatching)
{
    CloudXYZPtr cld = sig_.MostRecent(NAME_INPUT_POINT_CLOUD);
    for(size_t i=0; i<layers_.size()-1; i++) {

        LayerFactory::LayerShared l = layers_[i];
        l->Activate(sig_);
        l->Response(sig_);

        Triangles t = sig_.MostRecent(NAME_OUTPUT_VERTICES).get<VecVertices>();

        ELM_COUT_VAR(PointCloud2Mat_(cld));

        std::cout<<format(VecVertices2Mat(t, false), Formatter::FMT_NUMPY)<<std::endl;

        Mat1f m;
        elm::TriangulatedCloudToAdjacencyX(cld, t, m);

        ELM_COUT_VAR(format(m, Formatter::FMT_NUMPY));
    }

}


} // annonymous namespace for test cases and test fixtures

#endif // __WITH_PCL
