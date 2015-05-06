/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_READMNISTLABELS_H_
#define _ELM_IO_READMNISTLABELS_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/io/base_readermnistfile.h"

namespace elm {

/**
 * @brief class for reading MNIST label data
 */
class ReadMNISTLabels : public base_ReaderMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2049; ///< used to validate file integrity, see MNIST site

    ReadMNISTLabels();

    virtual int MagicNumber() const;

    virtual cv::Mat Next();

    virtual void Next(Signal &signal);
};

} // namespace elm

#endif // _ELM_IO_READMNISTLABELS_H_
