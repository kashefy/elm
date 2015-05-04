/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/base_reader.h"

#include "elm/ts/mat_assertions.h"

#include <boost/filesystem.hpp>

#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace elm;

class DummyReader : public base_Reader
{

};

class ReaderTest : public ::testing::Test
{
protected:
    virtual void SetUp() {

    }
};

TEST_F(ReaderTest, Constructor) {

}
