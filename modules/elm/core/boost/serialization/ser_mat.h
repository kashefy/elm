/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_H_
#define _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_H_

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/array.hpp>

#include <opencv2/core/core.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
namespace boost {
namespace serialization {

    /**
     * @brief Serialize OpenCV Mat
     * Inspired by https://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
     * @param ar archive
     * @param obj object to serialize
     * @param version
     */
    template<class Tarchive>
    void save(Tarchive &ar, const cv::Mat &obj, const unsigned int version)
    {
        using namespace boost::serialization;
        size_t elem_size = obj.elemSize();
        size_t elem_type = obj.type();

        ar & make_nvp("rows", obj.rows);
        ar & make_nvp("cols", obj.cols);
        ar & make_nvp("elemSize", elem_size);
        ar & make_nvp("type", elem_type);

        const size_t data_size = obj.cols * obj.rows * elem_size;
        ar & make_nvp("data", make_array(obj.ptr(), data_size));
    }

    /**
     * @brief De-serialize OpenCV Mat
     * Inspired by https://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
     * @param[in] ar archive
     * @param[out] obj object loaded from archive
     * @param[in] version
     */
    template<class Tarchive>
    void load(Tarchive & ar, ::cv::Mat& obj, const unsigned int version)
    {
        int cols, rows;
        size_t elem_size, elem_type;

        ar & rows;
        ar & cols;
        ar & elem_size;
        ar & elem_type;

        obj.create(rows, cols, elem_type);

        size_t data_size = obj.cols * obj.rows * elem_size;
        ar & boost::serialization::make_array(obj.ptr(), data_size);
    }

} // namespace boost
} // namespace serialization

#endif // _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_H_
