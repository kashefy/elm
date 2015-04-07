/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_READNYUDEPTHV2LABELED_H_
#define _ELM_IO_READNYUDEPTHV2LABELED_H_

#include <string>

#include <opencv2/core/core.hpp>

#include "elm/core/cv/typedefs_fwd.h"

namespace elm {

class MatlabMATFileReader;

/**
 * @brief class for reading the NYU Depth v2 labeled database
 * @cite SilbermanECCV12
 */
class ReadNYUDepthV2Labeled
{
public:
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

    /**
     * @brief Check if we've reached the end of the file
     * @return true on end of file reached
     */
    virtual bool IS_EOF() const;

protected:
    static const std::string KEY_DEPTHS;
    static const std::string KEY_IMAGES;
    static const std::string KEY_LABELS;

    MatlabMATFileReader *reader_;
    int index_;
    int nb_items_;

    cv::Mat images_;
    cv::Mat labels_;
    cv::Mat depths_;
};

} // namespace elm

#endif // _ELM_IO_READNYUDEPTHV2LABELED_H_
