/** forward declarations of STL data types
 */
#ifndef SEM_CORE_STL_TYPEDEFS_FWD_H_
#define SEM_CORE_STL_TYPEDEFS_FWD_H_

#include <string>
#include <vector>

typedef unsigned char uchar;

//namespace std {

//class string;

//template <typename T> class vector;

//} // fake namespace STL for fwd declaration

typedef std::vector< std::string > VecS;    ///< Convinience typedef for vector of strings
typedef std::vector< float > VecF;          ///< Convinience typedef for vector of floats

#endif // SEM_CORE_STL_TYPEDEFS_FWD_H_
