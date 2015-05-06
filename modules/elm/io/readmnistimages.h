/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_READMNISTIMAGES_H_
#define _ELM_IO_READMNISTIMAGES_H_

#include <opencv2/core/core.hpp>

#include "elm/io/base_readermnistfile.h"

namespace elm {

/**
 * @brief class for Reading MNIST image data
 */
class ReadMNISTImages : public base_ReaderMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2051; ///< used to validate file integrity, see MNIST site

    ReadMNISTImages();

    virtual int ReadHeader(const std::string &path);

    virtual int MagicNumber() const;

    virtual cv::Mat Next();

    virtual void Next(Signal &signal);

protected:
    int rows_;   ///< no. of rows per image
    int cols_;   ///< no. of columns per image
};

class ReadMNISTImagesTransl : public ReadMNISTImages
{
public:
    static const int MAGIC_NUMBER = 2051; ///< used to validate file integrity, see MNIST site

    ReadMNISTImagesTransl();

    virtual int ReadHeader(const std::string &path);

    virtual cv::Mat Next();

    /**
     * @brief Set larger scene dimensions
     * @param rows
     * @param cols
     */
    virtual void SceneDims(int rows, int cols);

    /**
     * @brief Get location of most recently loaded object image
     * @return Rect around most recently loaded object image
     */
    virtual cv::Rect2i Location() const;

protected:

    virtual bool IsSceneDimsSet() const;

    cv::Size2i scene_dims_;     ///< scene dimensions
    cv::Point2i current_loc_;   ///< top-left of most recent location
};

} // namespace elm

#endif // _ELM_IO_READMNISTIMAGES_H_
