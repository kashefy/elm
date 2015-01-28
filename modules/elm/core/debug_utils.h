/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_CORE_DEBUG_UTILS_H_
#define ELM_CORE_DEBUG_UTILS_H_

#include <iostream>

namespace elm {

/** @brief Macro for easy printing.
 * Prints:
 * <variable name>:newline
 * <value>newline
 * @param[in] variable to print.
  */
#define COUT_VAR(x) std::cout << #x << ":" << std::endl << x << std::endl;

/** @brief Macro to print two variables/expressions in one call.
 * A little something for the lazy.
 */
#define COUT_VAR(x, y) COUT_VAR(x); COUT_VAR(y)

} // namespace elm

#endif // ELM_CORE_DEBUG_UTILS_H_
