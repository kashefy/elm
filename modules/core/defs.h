/** @file define a few macros and constants here
 *  @todo how to suppress "defined but unused" warning of NA_IDX
 */
#ifndef SEM_CORE_DEFS_H_
#define SEM_CORE_DEFS_H_

#define SEM_PI2   1.57079632679

/** easily cast std::unique_ptr to a derived type
 */
#define SEM_DYN_CAST(type, ptr) dynamic_cast<type*>(ptr.get())

// define constants
namespace sem
{
static int NA_IDX   =   -1;   ///< non-applicable index

}

#endif // SEM_CORE_DEFS_H_
