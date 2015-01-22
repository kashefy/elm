/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** @file define a few macros and constants here
 *  @todo how to suppress "defined but unused" warning of NA_IDX
 */
#ifndef ELM_CORE_DEFS_H_
#define ELM_CORE_DEFS_H_

#define ELM_PI2   1.57079632679

/** easily cast std::unique_ptr to a derived type
 */
#define ELM_DYN_CAST(type, ptr) dynamic_cast<type*>(ptr.get())

// define constants
namespace elm
{
extern int NA_IDX;  ///< = -1; non-applicable index

}

#endif // ELM_CORE_DEFS_H_
