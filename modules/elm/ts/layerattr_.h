/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_TS_LAYERATTR__H_
#define ELM_TS_LAYERATTR__H_

#include <map>
#include <string>

class base_Layer;

 namespace elm {

enum LayerIOKeyType
{
    INPUT,
    OUTPUT
};

typedef std::pair<bool, std::string> IOName; ///< convinience typedef to layer io name with bool indicating input (0) or output (1)
typedef std::map< std::string, std::pair<bool, std::string> > MapIONamesB; ///< convinience typedef for a map of io keys and corresponding io name

typedef std::map< std::string, std::pair<LayerIOKeyType, std::string> > MapIONames; ///< convinience typedef for a map of io keys and corresponding io name



/**
 * @brief Layer Attributes struct, a helper struct for lyaer p-typed test cases.
 * It enables defining values to be used inside the tests
 *
 * Caution when defining members for you layer type:
 *
 * - statc members such as io_pairs depend on static initializations in the layer.
 *   Initilizing them in the layer's unittest source file, leads to a "static initialization order fiasco"
 *   To avoid this, initialize the LayerAttr_ member for your layer type in your layer's implementation file.
 *   Preferrably inside __WITH_GTEST define-guards.
 *
 * These values are set once per type before INSTANTIATE_TYPED_TEST_CASE_P(...)
 * in subscribing layer's unittest source file.
 */
template<class TLayer>
struct LayerAttr_
{
    static elm::MapIONames io_pairs; ///< needs to be initialized in layer's implementation file.
};

/** Macros for populating LayerAttr_::io_pairs member by client layer in its respective unittest source file
 */
#define ELM_ADD_INPUT_PAIR(key)   ( key, std::make_pair(elm::LayerIOKeyType::INPUT, "n"+key) )
#define ELM_ADD_OUTPUT_PAIR(key)  ( key, std::make_pair(elm::LayerIOKeyType::OUTPUT, "n"+key) )


} // namespace elm

#endif // ELM_TS_LAYERATTR__H_
