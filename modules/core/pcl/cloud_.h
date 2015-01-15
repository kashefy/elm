#ifndef SEM_CORE_PCL_CLOUD__H
#define SEM_CORE_PCL_CLOUD__H

#include <pcl/point_cloud.h>

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;
using namespace pcl;

namespace sem {

template <class TPoint>
typename pcl::PointCloud<TPoint >::Ptr Mat2PointCloudTP(const cv::Mat1f &m)
{
    // preemt code clutter due to all the different namespaces and types
    using namespace pcl;

    typedef PointCloud<TPoint> CloudTP;
    typedef typename PointCloud<TPoint>::iterator CloudTPIter;
    typename CloudTP::Ptr cloud_ptr;

    int nb_channels = m.channels();
    bool is_empty = m.empty();

    if(nb_channels == 1 && is_empty) {

        cloud_ptr.reset(new CloudTP);;
    }
    else if(nb_channels == 1 && (m.cols % 3 == 0)) {

        // determine width and height of new poValuesint cloud
        cloud_ptr.reset(new CloudTP(m.cols/3, m.rows));

        // populate
        size_t i=0;
        for(CloudTPIter itr=cloud_ptr->begin(); i<m.total(); ++itr, i+=3) {

            *itr = (TPoint(m(i), m(i+1), m(i+2)));
        }
    }
    else if(nb_channels == 1 && (m.cols % 4 == 0)) {

        // determine width and height of new poValuesint cloud
        cloud_ptr.reset(new CloudTP(m.cols/4, m.rows));

        // populate
        size_t i=0;
        for(CloudTPIter itr=cloud_ptr->begin(); i<m.total(); ++itr, i+=4) {

            *itr = (TPoint(m(i), m(i+1), m(i+2))); // ignore last coordinate
        }
    }
    else if(m.channels()==3) { // 3 channel matrix

        if(m.empty()) {
            cloud_ptr.reset(new CloudTP);
        }
        else {
            // widht and heigh of new point cloud directly follows source mat dims
            cloud_ptr.reset(new CloudTP(m.cols, m.rows));
        }

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                Vec3f p = m.at<Vec3f>(r, c);
                cloud_ptr->push_back(TPoint(p[0], p[1], p[2]));
            }
        }
    }
    else if(m.channels()==4) { // 4-channel matrix

        if(m.empty()) {
            cloud_ptr.reset(new CloudTP);
        }
        else {
            // widht and heigh of new point cloud directly follows source mat dims
            cloud_ptr.reset(new CloudTP(m.cols, m.rows));
        }

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                Vec4f p = m.at<Vec4f>(r, c);
                cloud_ptr->push_back(TPoint(p[0], p[1], p[2]));
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

} // namespace sem

#endif // CLOUD__H
