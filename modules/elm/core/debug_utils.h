/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_DEBUG_UTILS_H_
#define _ELM_CORE_DEBUG_UTILS_H_

#include <iostream>

namespace elm {


/** @brief Macro for easy printing.
 * Prints:
 * <variable name>:newline
 * <value>newline
 * @param[in] variable to print.
  */
#define ELM_COUT_VAR(x) std::cout << #x << ":" << std::endl << x << std::endl;

} // namespace elm

#endif // _ELM_CORE_DEBUG_UTILS_H_
