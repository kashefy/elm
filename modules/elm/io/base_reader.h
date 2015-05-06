/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_BASE_READER_H_
#define _ELM_IO_BASE_READER_H_

#include <string>

#include "elm/core/base_Layer.h"

namespace elm {

namespace detail {

const std::string BASE_READER_PARAM_PATH = "path";

} // namespace detail

class base_Reader : public base_Layer
{
public:
    static const std::string PARAM_PATH;    ///< path parameter

    virtual ~base_Reader();

    // reader methods
    /**
     * @brief read file header
     * @param path
     * @return no. of items
     */
    virtual int ReadHeader(const std::string &path) = 0;

    /**
     * @brief Check if we've reached the end of the file
     * @return true on end of file reached
     */
    virtual bool Is_EOF() const;

    /**
     * @brief get no. of items left
     * @return no. of items left
     */
    virtual int Nb_Items() const;

    virtual void Next(Signal &signal) = 0;

    // layer methods
    virtual void Clear();

    virtual void Reconfigure(const LayerConfig& config);

    virtual void InputNames(const LayerInputNames& io);

    virtual void OutputNames(const LayerOutputNames& io) = 0;

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    base_Reader();

    // members
    int nb_items_;  ///< no. of items left
};

} // namespace elm

#endif // _ELM_IO_BASE_READER_H_
