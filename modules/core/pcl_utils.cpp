#include "core/pcl_utils.h"

#ifdef __WITH_PCL

#include "core/exception.h"
#include "core/mat_utils.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

CloudXYZ::Ptr sem::Mat2PointCloud(const Mat_<float> &m)
{
    CloudXYZ::Ptr cloud_ptr;

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

Mat1f sem::PointCloud2Mat(CloudXYZ::Ptr &cloud_ptr)
{
    PointXYZ *points_ptr = cloud_ptr->points.data();
    return Mat1f(cloud_ptr->height, cloud_ptr->width*4, reinterpret_cast<float*>(points_ptr));
}

vector<Vertices > sem::Mat2VecVertices(const Mat &m)
{
    vector<Vertices > vv;

    if(!m.empty()) {

        vv.reserve(m.rows);

        int nb_channels = m.channels();
        int len_vertices;
        if(nb_channels==1) {

            // single-channel matrix, treat columns as vertices values
            len_vertices = m.cols;
        }
        else {
            // multi-channel matrix, treat channels as vertices values
            if(m.cols > 1) {

                SEM_THROW_BAD_DIMS("Cannot extract vertices from multi-channel multi-column matrix.");
            }

            len_vertices = nb_channels;
        }

        for(int i=0; i<m.rows; i++) {

            Vertices v;
            v.vertices.reserve(size_t(len_vertices));
            for(int j=0; j<len_vertices; j++) {

                v.vertices.push_back(m.at<float>(i, j));
            }
            vv.push_back(v);
        }
    }

    return vv;
}

Mat1f sem::VecVertices2Mat(const vector<Vertices >& vv, bool do_row_mat)
{
    Mat1f m;
    if(vv.size() > 0) {

        int nb_vertices = static_cast<int>(vv.size());
        int sz_vertex = static_cast<int>(vv[0].vertices.size());

        int mat_rows = 1;
        int mat_cols = nb_vertices;

        if(do_row_mat) {
            mat_cols *= sz_vertex;
        }
        else {

            mat_rows = nb_vertices;
        }

        m = Mat1f(mat_rows, mat_cols);
        int k=0;
        for(int i=0; i<nb_vertices; i++) {

            vector<uint32_t> tmp = vv[i].vertices;
            for(int j=0; j<sz_vertex; j++) {

                m(k++) = static_cast<float>(tmp[j]);
            }
        }
    }

    return m;
}

#endif // __WITH_PCL

