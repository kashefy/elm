/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_TS_FAKEMNISTIMAGESWRITER_H_
#define _ELM_TS_FAKEMNISTIMAGESWRITER_H_

#include "elm/ts/fakemnistlabelswriter.h"

namespace elm {

class FakeMNISTImagesWriter : public FakeMNISTLabelsWriter
{
public:
    static const int NB_ITEMS = 10;
    static const int ROWS = 3;
    static const int COLS = 4;

    FakeMNISTImagesWriter(const boost::filesystem::path& p);

    /**
     * @brief Save fake data to file
     * magic number
     * total no. of items
     * series label values [0,10)
     */
    void SaveHeader(int magic_number);

    virtual void SaveItems();
};

}

#endif // _ELM_TS_FAKEMNISTIMAGESWRITER_H_
