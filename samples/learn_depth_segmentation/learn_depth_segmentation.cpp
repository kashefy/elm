#include <iostream>

#include <boost/filesystem.hpp>

#include "elm/core/core.h"
#include "elm/core/debug_utils.h"
#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"
#include "elm/io/readnyudepthv2labeled.h"
#include "elm/layers/layerfactory.h"
#include "elm/layers/mlp.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace elm;

int main() {

    cout<<elm::GetVersion()<<endl;

    return 0;
}
