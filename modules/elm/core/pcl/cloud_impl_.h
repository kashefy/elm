/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_PCL_CLOUD_IMPL__H_
#define _ELM_CORE_PCL_CLOUD_IMPL__H_

#ifdef __WITH_PCL // the following template Converter Cloud Mat class requires PCL support

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>

#include "elm/core/exception.h"
#include "elm/core/pcl/point_traits.h"

extern template class pcl::PointCloud<pcl::PointXYZ >;
extern template class pcl::PointCloud<pcl::Normal >;
extern template class pcl::PointCloud<pcl::PointNormal >;

extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::Normal > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > >;

extern template class cv::Mat_<float>;

namespace elm {

/**
 * @brief Template class with utilities for Cloud-Mat conversions depending on Point type
 */
template <class TPoint>
class ConverterCloudMat_
{
public:
    typedef PCLPointTraits_<TPoint> PointTraitsTP;
    /**
     * @brief Copy Mat data to cloud
     * @param src Mat
     * @param step (no. of floats per element)
     * @param dst cloud
     * @throw elm::ExceptionTypeError for non-continuous Mat input
     */
    static void CopyContMatData2Cloud(const cv::Mat1f &src, size_t step, typename pcl::PointCloud<TPoint >::Ptr dst)
    {
        if(src.isContinuous()) {

            float *mat_data_ptr = reinterpret_cast<float*>(src.data);
            size_t step_size = sizeof(float)*step;
            for(typename pcl::PointCloud<TPoint>::iterator itr=dst->begin();
                    itr != dst->end(); ++itr) {

                //memcpy((*itr).data, mat_data_ptr, step_size); // Normal has data_n[] member, not data[]
                memcpy(&(*itr), mat_data_ptr, step_size);       // dangerous
                mat_data_ptr += step;
            }
        }
        else {
            ELM_THROW_TYPE_ERROR("Mat must be continuous.");
        }
    }

    /**
     * @brief Copy Mat data to cloud for non-continuous Mat objects (slower)
     * @param src Mat
     * @param step (no. of floats per element)
     * @param dst cloud
     * @todo test test test
     */
    static void CopyMatData2Cloud(const cv::Mat1f &src, size_t step, typename pcl::PointCloud<TPoint >::Ptr dst)
    {
        if(src.isContinuous()) {

            ConverterCloudMat_::CopyContMatData2Cloud(src, step, dst);
        }
        else {

            const float *mat_data_ptr = reinterpret_cast<const float*>(src.data);
            size_t step_size = sizeof(float)*step;

            int row = 0;
            int col_offset = 0;

            for(typename pcl::PointCloud<TPoint>::iterator itr=dst->begin();
                    itr != dst->end(); ++itr) {

                if(col_offset == src.cols) {

                    mat_data_ptr = src.ptr<float>(++row);
                    col_offset = 0;
                }

                //memcpy((*itr).data, mat_data_ptr, step_size); // Normal has data_n[] member, not data[]
                memcpy(&(*itr), mat_data_ptr, step_size);       // dangerous
                mat_data_ptr += step;

                col_offset += step;
            }
        }
    }

    /**
     * @brief Determine if Mat contains padded point cloud data
     * Padding does not need to be inferred from multi-channel matrices due to the limited cap on no. of channels
     * @param m single-channel Mat
     * @return true if Mat contains padded point cloud data
     */
    static bool IsPaddedMat(const cv::Mat1f &m) {

        if(m.empty()) {
            // nothing to do here
            return false;
        }

        size_t field_count = PointTraitsTP::FieldCount();
        size_t sz_point = PointTraitsTP::NbFloats();

        bool is_multi_field_count = m.cols % field_count == 0;
        bool is_multi_sz_point = m.cols % sz_point == 0;
        if(is_multi_field_count && !is_multi_sz_point) {

            return false;
        }
        else if(!is_multi_field_count && is_multi_sz_point) {

            return true;
        }
        else if(is_multi_field_count && is_multi_sz_point) { // ambiguous case

            // sniff for padding pattern
            size_t sz_padding = sz_point - field_count;

            // can't look at too many rows, that would slow things down too much
            const int SUB_ROWS = std::min(m.rows, 3);

            cv::Mat1f m_sub_rows = m.rowRange(0, SUB_ROWS);

            bool is_padded = true;
            for(size_t i=0, col=field_count; i<sz_padding && is_padded; i++, col++) {

                is_padded = sum(m_sub_rows.col(col))[0] == static_cast<float>(SUB_ROWS)*m_sub_rows(col);
            }
            return is_padded;
        }
        else {

            std::stringstream s;
            s << "Cannot determine padding for this Mat " <<
                 "no. of cols must be a multiple of either points field count or no. of floats occupied " <<
                 "(with or without padding).";
            ELM_THROW_BAD_DIMS(s.str());
        }
    }

    /** @brief Convert Single-channel Mat of floats to template PointCloud
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
     * @param m single-channel Mat of floats with point cloud data
     * @return PointCloud after conversion
     * @todo iron out multiple of field count and padded size ambiguity
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

        size_t field_count = PointTraitsTP::FieldCount();
        size_t sz_point = PointTraitsTP::NbFloats();

        bool is_padded = ConverterCloudMat_::IsPaddedMat(m);

        if(m.empty()) { // 1. empty
            cloud_ptr.reset(new CloudTP);
        }
        else if((m.cols % field_count == 0) && !is_padded) { // no padding

            // determine width and height of new point cloud
            cloud_ptr.reset(new CloudTP(m.cols/static_cast<int>(field_count), m.rows));
            ConverterCloudMat_::CopyMatData2Cloud(m, field_count, cloud_ptr);
        }
        else if((m.cols % sz_point == 0) && is_padded) { // with padding

            // determine width and height of new point cloud
            cloud_ptr.reset(new CloudTP(m.cols/static_cast<int>(sz_point), m.rows));
            ConverterCloudMat_::CopyMatData2Cloud(m, sz_point, cloud_ptr);
        }
        else {

            stringstream s;
            s << "Failed to convert this matrix to a point cloud." <<
                 " No. of Mat columns must be a multiple of the Point's field count if not padded,"<<
                 " or no. of floats occupied by point element if mat is padded accordingly.";
            ELM_THROW_BAD_DIMS(s.str());
        }

        return cloud_ptr;
    }

    /**
     * @brief the multi-channel version
     */
    template<int ch>
    static typename pcl::PointCloud<TPoint >::Ptr Mat2PointCloud(const cv::Mat_<cv::Vec<float, ch> > &m)
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
        size_t field_count = PointTraitsTP::FieldCount();
        size_t sz_point = PointTraitsTP::NbFloats();

        if(m.empty()) { // 1. empty
            cloud_ptr.reset(new CloudTP);
        }
        else if(nb_channels==static_cast<int>(field_count) || nb_channels==static_cast<int>(sz_point)) { // n-channel matrix

            cloud_ptr.reset(new CloudTP(m.cols, m.rows));
            ConverterCloudMat_::CopyMatData2Cloud(m, static_cast<size_t>(nb_channels), cloud_ptr);
        }
        else {

            stringstream s;
            s << "Failed to convert this matrix to a point cloud." <<
                 " No. of Mat columns must be a multiple of the Point's field count if not padded,"<<
                 " or no. of floats occupied by point element if mat is padded accordingly.";
            ELM_THROW_BAD_DIMS(s.str());
        }

        return cloud_ptr;
    }
};

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_CORE_PCL_CLOUD_IMPL__H_
