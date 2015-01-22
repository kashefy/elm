#include "sem/ts/layer_assertions.h"

#include "gtest/gtest.h"

#include "sem/core/base_Layer.h"
#include "sem/core/exception.h"
#include "sem/core/layerconfig.h"

using namespace std;
using namespace elm;

/** class for deriving from base Layer for test purposes
  */
class DummyChildLayer : public base_Layer
{
public:
    virtual void Clear() {}

    virtual void Reconfigure(const LayerConfig &config) {}

    virtual void IONames(const LayerIONames &config) {}

    virtual void Activate(const Signal &signal) {}

    virtual void Apply() {}

    virtual void Response(Signal &signal) {}

    DummyChildLayer() {}
};

TEST(LayerAssertionsNoReqIONamesTest, EmptyIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayer());
    EXPECT_NO_THROW(to_ptr->IONames(LayerIONames()));
}

TEST(LayerAssertionsReqIONamesTest, NoRequiredIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayer());
    ValidateRequiredIONames(MapIONames(), to_ptr);
}

const string NAME_IN        = "i";
const string NAME_OUT       = "o";
const string NAME_OPT_OUT   = "oo";

class DummyChildLayerWithReqIONames : public DummyChildLayer
{
public:
    // I/O Names
    static const string KEY_INPUT_IN;
    static const string KEY_OUTPUT_OUT;
    static const string KEY_OUTPUT_OPT_OUT;

    virtual void IONames(const LayerIONames &config) {

        name_in_    = config.Input(KEY_INPUT_IN);
        name_out_   = config.Output(KEY_OUTPUT_OUT);
        name_opt_out_ = config.OutputOpt(KEY_OUTPUT_OPT_OUT);
    }

    DummyChildLayerWithReqIONames() : DummyChildLayer() {}

    // members
    string name_in_;
    string name_out_;
    OptS name_opt_out_;
};
const string DummyChildLayerWithReqIONames::KEY_INPUT_IN        = "in";
const string DummyChildLayerWithReqIONames::KEY_OUTPUT_OUT      = "out";
const string DummyChildLayerWithReqIONames::KEY_OUTPUT_OPT_OUT  = "optout";


TEST(LayerAssertionsReqIONamesTest, EmptyIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayerWithReqIONames());
    EXPECT_THROW(to_ptr->IONames(LayerIONames()), ExceptionKeyError);
}

TEST(LayerAssertionsReqIONamesTest, RequiredIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayerWithReqIONames());

    map<string, pair<bool, string> > io_pairs; // false for input, true for output
    io_pairs[DummyChildLayerWithReqIONames::KEY_INPUT_IN    ] = make_pair(0, NAME_IN);
    io_pairs[DummyChildLayerWithReqIONames::KEY_OUTPUT_OUT ] = make_pair(1, NAME_OUT);

    ValidateRequiredIONames(io_pairs, to_ptr);
}

/**
 * @brief test with required + optional IO names
 */
TEST(LayerAssertionsReqIONamesTest, AllIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayerWithReqIONames());

    map<string, pair<bool, string> > io_pairs; // false for input, true for output
    io_pairs[DummyChildLayerWithReqIONames::KEY_INPUT_IN    ] = make_pair(0, NAME_IN);
    io_pairs[DummyChildLayerWithReqIONames::KEY_OUTPUT_OUT  ] = make_pair(1, NAME_OUT);
    io_pairs[DummyChildLayerWithReqIONames::KEY_OUTPUT_OPT_OUT ] = make_pair(1, NAME_OPT_OUT);

    ValidateRequiredIONames(MapIONames(), to_ptr);
}
