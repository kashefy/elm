/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_MATLABMATFILEREADER_H_
#define _ELM_IO_MATLABMATFILEREADER_H_

#ifdef __WITH_MATIO

#include <vector>
#include <string>

#include "elm/core/cv/typedefs_fwd.h"

struct _mat_t;
typedef struct _mat_t mat_t;
struct matvar_t;

namespace elm {

/**
 * @brief Class to wrap around MAT file reading library
 */
class MatlabMATFileReader
{
public:
    MatlabMATFileReader();

    virtual ~MatlabMATFileReader();

    /**
     * @brief Gather info on MAT-file content
     * @param path
     * @throws elm::ExceptionFileIO error on invalid path and or invalid extension
     */
    void ReadHeader(const std::string &path);

    /**
     * @brief get list of top-level variable names
     * @return list of top-level variable names
     */
    std::vector<std::string> TopLevelVarNames() const;

    /**
     * @brief Seek to a specific variable name
     * @param var_name name of variable requested
     * @throw elm::EceptionKeyError on non-existent variable
     */
    void Seek(const std::string& var_name);

    /**
     * @brief get Cursor pointing to variable data
     * @return pointer to variable data
     */
    matvar_t* Cursor() const;

    cv::Mat CursorToMat() const;

protected:
    void ResetFileHandle();

    mat_t *matfp_;  ///< file handle

    std::vector<std::string> var_names_;    ///< list of top-level variable names extraced from file

    matvar_t* cursor_;  ///< cursor to current variable

    std::string path_;
};

}// namespace elm

#endif // __WITH_MATIO

#endif // _ELM_IO_MATLABMATFILEREADER_H_
