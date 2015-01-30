#include "elm/core/graph/adjacency.h"

#include <opencv2/core.hpp>

#include "elm/core/exception.h"

#ifdef __WITH_PCL

#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

Mat1f elm::TriangulatedCloudToAdjacencyMat(const CloudXYZPtr &cld, const Triangles &t)
{
    ELM_THROW_NOT_IMPLEMENTED;
//    int nb_vertices = static_cast<int>(cld->size());
//    Mat1f m(nb_vertices, nb_vertices);

//    for(Triangles::const_iterator itr=t.begin(); itr!=t.end(); ++itr) {

//        const VecVertices V=*itr;

//        if(V.size() < 3) {

//            ELM_THROW_BAD_DIMS("triangle with less than 3 vertices.");
//        }

//        //foreach vertex

//        pcl::PointXYZ p1 = this->meshCloud.points.at(V[0]);
//        pcl::PointXYZ p2 = this->meshCloud.points.at(V[3]);
//        pcl::PointXYZ p3 = this->meshCloud.points.at(V[2]);

//    }

//    return m;
}

#endif // __WITH_PCL
