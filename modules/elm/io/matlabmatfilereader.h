/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_MATLABMATFILEREADER_H_
#define _ELM_IO_MATLABMATFILEREADER_H_

#include <vector>
#include <string>

struct _mat_t;
typedef struct _mat_t mat_t;

namespace elm {

/**
 * @brief Class to wrap around MAT file reading library
 */
class MatlabMATFileReader
{
public:
    MatlabMATFileReader();

    virtual ~MatlabMATFileReader();

    void ReadHeader(const std::string &path);

    std::vector<std::string> TopLevelVarNames() const;

protected:
    mat_t *matfp_;  ///< file handle

    std::vector<std::string> var_names_;
};

}// namespace elm

#endif // _ELM_IO_MATLABMATFILEREADER_H_
