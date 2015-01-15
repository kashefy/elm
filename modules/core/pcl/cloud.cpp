#include "core/pcl/cloud.h"

#ifdef __WITH_PCL

#include "core/exception.h"
#include "core/cv/mat_utils.h"
#include <pcl/point_traits.h>
using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

CloudXYZPtr sem::Mat2PointCloud(const Mat_<float> &m)
{
    CloudXYZPtr cloud_ptr;

    int nb_channels = m.channels();
    bool is_empty = m.empty();

    if(nb_channels == 1 && is_empty) {

        cloud_ptr.reset(new CloudXYZ);;
    }
    else if(nb_channels == 1 && (m.cols % 3 == 0)) {

        // determine width and height of new poValuesint cloud
        cloud_ptr.reset(new CloudXYZ(m.cols/3, m.rows));

        // populate
        size_t i=0;
        for(CloudXYZ::iterator itr=cloud_ptr->begin(); i<m.total(); ++itr, i+=3) {

            *itr = (PointXYZ(m(i), m(i+1), m(i+2)));
        }
    }
    else if(nb_channels == 1 && (m.cols % 4 == 0)) {

        // determine width and height of new poValuesint cloud
        cloud_ptr.reset(new CloudXYZ(m.cols/4, m.rows));

        // populate
        size_t i=0;
        for(CloudXYZ::iterator itr=cloud_ptr->begin(); i<m.total(); ++itr, i+=4) {

            *itr = (PointXYZ(m(i), m(i+1), m(i+2))); // ignore last coordinate
        }
    }
    else if(m.channels()==3) { // 3 channel matrix

        if(m.empty()) {
            cloud_ptr.reset(new CloudXYZ);
        }
        else {
            // widht and heigh of new point cloud directly follows source mat dims
            cloud_ptr.reset(new CloudXYZ(m.cols, m.rows));
        }

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                Vec3f p = m.at<Vec3f>(r, c);
                cloud_ptr->push_back(PointXYZ(p[0], p[1], p[2]));
            }
        }
    }
    else if(m.channels()==4) { // 4-channel matrix

        if(m.empty()) {
            cloud_ptr.reset(new CloudXYZ);
        }
        else {
            // widht and heigh of new point cloud directly follows source mat dims
            cloud_ptr.reset(new CloudXYZ(m.cols, m.rows));
        }

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                Vec4f p = m.at<Vec4f>(r, c);
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

Mat1f sem::PointCloud2Mat(CloudXYZPtr &cloud_ptr)
{
    PointXYZ *points_ptr = cloud_ptr->points.data();
    return Mat1f(cloud_ptr->height, cloud_ptr->width*4, reinterpret_cast<float*>(points_ptr));
}

#endif // __WITH_PCL

