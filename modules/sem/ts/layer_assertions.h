/** @file routines to help test layer functionality
  */
#ifndef ELM_TS_LAYER_UTILS_H_
#define ELM_TS_LAYER_UTILS_H_

#include <map>
#include <memory>

class base_Layer;

namespace elm {

typedef std::pair<bool, std::string> IOName; ///< convinience typedef to layer io name with bool indicating input (0) or output (1)
typedef std::map< std::string, std::pair<bool, std::string> > MapIONames; ///< convinience typedef for a map of io keys and corresponding io name

/**
 * @brief test around a layer's validation of its required io names
 * Basically tests that elm::ExceptionKeyError is thrown whenever a I/O name is not provided.
 *
 * Generates various combinations of present and missing IO names
 *
 * @param map of io name pairs
 * @param pointer to layer under test
 * @todo rewrite to return AssertionResult
 */
void ValidateRequiredIONames(const MapIONames &io_pairs, std::shared_ptr<base_Layer> &layer_ptr);

}

#endif // ELM_TS_LAYER_UTILS_H_
