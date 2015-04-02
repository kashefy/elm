/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

#include "matio.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace elm;

class MatlabMATFileReaderTest : public ::testing::Test
{

};

TEST_F(MatlabMATFileReaderTest, MATIO)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    MatlabMATFileReader reader;
    reader.ReadHeader(p.string());

    vector<string> var_names = reader.TopLevelVarNames();

    for(size_t i=0; i<var_names.size(); i++) {

        ELM_COUT_VAR(var_names[i]);
    }
}
