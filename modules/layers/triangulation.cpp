#include "layers/triangulation.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <opencv2/core/eigen.hpp> // for eigen2cv(), must be preceeded definitio of Eigen either PCL or #include <Eigen/Dense>

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/mat_utils.h"
#include "core/pcl_utils.h"
#include "core/signal.h"

Triangulation::Triangulation()
{
}

#endif // __WITH_PCL
