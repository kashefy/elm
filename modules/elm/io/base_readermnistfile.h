/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_BASE_READERMNISTFILE_H_
#define _ELM_IO_BASE_READERMNISTFILE_H_

#include <fstream>

#include "elm/core/cv/typedefs_fwd.h"
#include "elm/io/base_reader.h"

namespace elm {

namespace detail {

const std::string BASE_READERMNISTFILE_KEY_OUTPUT = "out";

}

/**
 * @brief base class for reading MNIST files
 * @cite LeCun1998
 * @link http://yann.lecun.com/exdb/mnist/
 */
class base_ReaderMNISTFile : public base_Reader
{
public:
    static const std::string KEY_OUTPUT;

    virtual ~base_ReaderMNISTFile();

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
    base_ReaderMNISTFile();

    std::ifstream input_; ///< input stream

    std::string name_out_;  ///< destination name for labels in signal object
};

} // namespace elm

#endif // _ELM_IO_BASE_READERMNISTFILE_H_
