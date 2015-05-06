/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file Classes for reading MNIST database files
 */
#ifndef _ELM_IO_READMNIST_H_
#define _ELM_IO_READMNIST_H_

#include <fstream>

#include <opencv2/core/core.hpp>

#include "elm/core/typedefs_fwd.h"
#include "elm/io/base_reader.h"

namespace elm {

/**
 * @brief base class for reading MNIST files
 * @cite LeCun1998
 * @link http://yann.lecun.com/exdb/mnist/
 */
class base_ReadMNISTFile : public base_Reader
{
public:
    static const std::string KEY_OUTPUT;

    virtual ~base_ReadMNISTFile();

    /**
     * @brief Get magic number expected in file header
     * with which we verify the file's integrity
     * @return expected number in file header
     */
    virtual int MagicNumber() const = 0;

    /**
     * @brief read file header
     * @param path
     * @return no. of items
     */
    virtual int ReadHeader(const std::string &path);

    /**
     * @brief read next item and advance file handle
     * @return item value encapsulated in a matrix object
     */
    virtual cv::Mat Next() = 0;

    virtual void OutputNames(const LayerOutputNames& io);

protected:
    base_ReadMNISTFile();

    std::ifstream input_; ///< input stream

    std::string name_out_;  ///< destination name for labels in signal object
};

/**
 * @brief class for reading MNIST label data
 */
class ReadMNISTLabels : public base_ReadMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2049; ///< used to validate file integrity, see MNIST site

    ReadMNISTLabels();

    virtual int MagicNumber() const;

    virtual cv::Mat Next();

    virtual void Next(Signal &signal);
};

/**
 * @brief class for Reading MNIST image data
 */
class ReadMNISTImages : public base_ReadMNISTFile
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

#endif // _ELM_IO_READMNIST_H_
