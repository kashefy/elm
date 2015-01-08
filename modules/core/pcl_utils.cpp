#include "core/pcl_utils.h"

#ifdef __WITH_PCL

#include "core/exception.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

PointCloudXYZ::Ptr sem::Mat2PointCloud(const Mat_<float> &m)
{
    PointCloudXYZ::Ptr cloud_ptr;

    int nb_channels = m.channels();
    bool is_empty = m.empty();

    if(nb_channels == 1 && is_empty) {

        cloud_ptr.reset(new PointCloudXYZ);;
    }
    else if(nb_channels == 1 && (m.cols % 3 == 0)) {

        // determine width and height of new poValuesint cloud
        cloud_ptr.reset(new PointCloudXYZ(m.cols/3, m.rows));

        // populate
        size_t i=0;
        for(PointCloudXYZ::iterator itr=cloud_ptr->begin(); i<m.total(); ++itr, i+=3) {

            *itr = (PointXYZ(m(i), m(i+1), m(i+2)));
        }
    }
    // 3 channel matrix
    else if(m.channels()==3) {

        if(m.empty()) {
            cloud_ptr.reset(new PointCloudXYZ);
        }
        else {
            // widht and heigh of new point cloud directly follows source mat dims
            cloud_ptr.reset(new PointCloudXYZ(m.cols, m.rows));
        }

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                Vec3f p = m.at<Vec3f>(r, c);
                cloud_ptr->push_back(PointXYZ(p[0], p[1], p[2]));
            }
        }
    }
    else {
        stringstream s;
        s << "Failed to convert this matrix to point cloud." <<
             " No. of Mat columns must be a multiple of 3";
        SEM_THROW_BAD_DIMS(s.str());
    }

    return cloud_ptr;
}

Mat1f sem::PointCloud2Mat(PointCloudXYZ::Ptr &cloud_ptr)
{
    PointXYZ *points_ptr = cloud_ptr->points.data();
    return Mat1f(cloud_ptr->height, cloud_ptr->width*4, reinterpret_cast<float*>(points_ptr));
}

#endif // __WITH_PCL

