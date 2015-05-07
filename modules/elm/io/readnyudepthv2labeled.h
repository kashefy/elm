/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_READNYUDEPTHV2LABELED_H_
#define _ELM_IO_READNYUDEPTHV2LABELED_H_

#ifdef __WITH_MATIO

#include <string>

#include <opencv2/core/core.hpp>

#include "elm/io/base_reader.h"
#include "elm/core/cv/typedefs_fwd.h"

namespace elm {

class MatlabMATFileReader;

/**
 * @brief class for reading the NYU Depth v2 labeled database
 * @cite SilbermanECCV12
 */
class ReadNYUDepthV2Labeled : public base_Reader
{
public:
    static const std::string KEY_DEPTHS;
    static const std::string KEY_IMAGE;
    static const std::string KEY_LABELS;

    virtual ~ReadNYUDepthV2Labeled();

    ReadNYUDepthV2Labeled();

    /**
     * @brief read file header
     * @param path
     * @return no. of items
     */
    virtual int ReadHeader(const std::string &path);

    /**
     * @brief advance to next item
     */
    virtual void Next(cv::Mat &bgr, cv::Mat &depth, cv::Mat &labels);

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Next(Signal &signal);

protected:
    MatlabMATFileReader *reader_;

    cv::Mat images_;
    cv::Mat labels_;
    cv::Mat depths_;

    std::string name_out_depths_;
    std::string name_out_image_;
    std::string name_out_labels_;
};

} // namespace elm

#endif // __WITH_MATIO

#endif // _ELM_IO_READNYUDEPTHV2LABELED_H_
