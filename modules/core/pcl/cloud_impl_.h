#ifndef SEM_CORE_PCL_CLOUD_IMPL__H_
#define SEM_CORE_PCL_CLOUD_IMPL__H_

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>      // getFields()

#include "core/exception.h"

namespace sem {

/** @brief get no. of fields of a pcl point type.
 * This does not account for SSE alignment or mixed types
 * @return field count
 */
template <class TPoint>
class ConverterCloudMat_
{
public:
    static size_t FieldCount()
    {
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields<TPoint>(fields);

//        for(size_t i=0; i<fields.size(); i++)
//        {
//            pcl::PCLPointField f = fields[i];
//            std::cout<<f.name<<",";
//        }
//        std::cout<<std::endl;

        return fields.size();
    }

    /** @brief Get no. of floats occupied by Point struct
     * @return no. of floats occupied by point struct
     */
    static size_t NbFloats()
    {
        static_assert(sizeof(TPoint) % sizeof(float) == 0, "This point type has non-float components");
        return sizeof(TPoint) / sizeof(float);
    }

    static void CopyMatData2Cloud(const cv::Mat1f &src, size_t step, typename pcl::PointCloud<TPoint >::Ptr dst)
    {
        float *mat_data_ptr = reinterpret_cast<float*>(src.data);
        size_t step_size = sizeof(float)*step;
        for(typename pcl::PointCloud<TPoint>::iterator itr=dst->begin();
                itr != dst->end(); ++itr) {

            //memcpy((*itr).data, mat_data_ptr, step_size); // Normal has data_n[] member, not data[]
            memcpy(&(*itr), mat_data_ptr, step_size);       // dangerous
            mat_data_ptr += step;
        }
    }

    /** @brief Convert Mat of floats to template PointCloud
     *
     * This conversion involves a deep copy.
     *
     * logic followed:
     *
     * 1. empty Mat -> produce an empty PointCloud
     * 2. single-channel Mat columns are divisible by no. of point fields, mat data is not pre-padded
     * 3. single-channel Mat columns are divisible by no. of points' float capacity, mat contains padded data
     * 4. multi-channel Mat...
     *
     * @param mat with point cloud data
     * @return PointCloud after conversion
     */
    static typename pcl::PointCloud<TPoint >::Ptr Mat2PointCloud(const cv::Mat1f &m)
    {
        // preemt code clutter due to all the different namespaces and types
        using namespace std;
        using namespace cv;
        using namespace pcl;

        typedef PointCloud<TPoint> CloudTP;
        typedef typename PointCloud<TPoint>::iterator CloudTPIter;

        // all types and namespaces defined.

        typename CloudTP::Ptr cloud_ptr;

        int nb_channels = m.channels();
        size_t field_count = ConverterCloudMat_::FieldCount();
        size_t sz_point = ConverterCloudMat_::NbFloats();

        if(m.empty()) { // 1. empty
            cloud_ptr.reset(new CloudTP);
        }
        else if(nb_channels == 1) {

            if(m.cols % field_count == 0) { // 2. no padding

                // determine width and height of new point cloud
                cloud_ptr.reset(new CloudTP(m.cols/static_cast<int>(field_count), m.rows));
                ConverterCloudMat_::CopyMatData2Cloud(m, field_count, cloud_ptr);
            }
            else if(m.cols % sz_point == 0) {

                // determine width and height of new point cloud
                cloud_ptr.reset(new CloudTP(m.cols/static_cast<int>(sz_point), m.rows));
                ConverterCloudMat_::CopyMatData2Cloud(m, sz_point, cloud_ptr);
            }
            else {

                stringstream s;
                s << "Failed to convert this matrix to a point cloud." <<
                     " No. of Mat columns must be a multiple of the Point's field count "<<
                     "(with or without padding).";
                SEM_THROW_BAD_DIMS(s.str());
            }
        }
        else if(nb_channels==static_cast<int>(field_count) || nb_channels==static_cast<int>(sz_point)) { // n-channel matrix

            cloud_ptr.reset(new CloudTP(m.cols, m.rows));
            ConverterCloudMat_::CopyMatData2Cloud(m, static_cast<size_t>(nb_channels), cloud_ptr);
        }
        else {

            stringstream s;
            s << "Failed to convert this matrix to point cloud." <<
                 " No. of Mat channels must be a multiple of the Point type's field count "<<
                 "(with or without padding).";
            SEM_THROW_BAD_DIMS(s.str());
        }

        return cloud_ptr;
    }
};

} // namespace sem

#endif // SEM_CORE_PCL_CLOUD_IMPL__H_
