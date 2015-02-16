/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** Classes for reading MNIST database files
 *  source: http://yann.lecun.com/exdb/mnist/
 */
#ifndef _ELM_IO_READMNIST_H_
#define _ELM_IO_READMNIST_H_

#include <string>
#include <fstream>

#include <opencv2/core/core.hpp>

#include "elm/core/typedefs_fwd.h"

/**
 * @brief base class for reading MNIST files
 */
class base_ReadMNISTFile
{
public:
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
     * @brief read next item
     * @return item value encapsulated in a matrix object
     */
    virtual cv::Mat Next() = 0;

    /**
     * @brief Check if we've reached the end of the file
     * @return true on end of file reached
     */
    virtual bool IS_EOF() const;

protected:
    base_ReadMNISTFile();

protected:
    std::ifstream input_; ///< input stream
    int nb_items_;        ///< total no. of items
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
};

class ReadMNISTImages : public base_ReadMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2051; ///< used to validate file integrity, see MNIST site

    ReadMNISTImages();

    virtual int ReadHeader(const std::string &path);

    virtual int MagicNumber() const;

    virtual cv::Mat Next();

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

    cv::Size2i scene_dims_;   ///< scene dimensions

    cv::Point2i current_loc_;    ///< top-left of most recent location
};

#endif // _ELM_IO_READMNIST_H_
