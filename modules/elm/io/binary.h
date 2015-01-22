/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_IO_BINARY_H_
#define ELM_IO_BINARY_H_

#include <algorithm>
#include <limits.h>
#include <stdint.h>

/**
 * Macros and enum for checking host endianness
 * source: http://stackoverflow.com/questions/2100331/c-macro-definition-to-determine-big-endian-or-little-endian-machine
 */
#if CHAR_BIT != 8
    #error "unsupported char size"
#endif // CHAR_BIT != 8

/**
 * Enum for 32-bit endian type
 */
enum
{
    O32_LITTLE_ENDIAN   = 0x03020100ul,
    O32_BIG_ENDIAN      = 0x00010203ul,
    O32_PDP_ENDIAN      = 0x01000302ul
};

static const union { unsigned char bytes[4]; uint32_t value; } o32_host_order =
    { { 0, 1, 2, 3 } };

#define O32_HOST_ORDER (o32_host_order.value)
#define IS_32_ENDIAN(x) O32_HOST_ORDER == x
#define IS_32_LITTLE_ENDIAN O32_HOST_ORDER == O32_LITTLE_ENDIAN

namespace elm
{

/**
 * Perform Endian Swap
 * Useful when integers in binary files are stored in the MSB first (high endian) format used by most non-Intel processors,
 * but read on a platform with an Intel processor and other low-endian machines. Those have to flip the bytes of those integers.
 *
 * source: http://stackoverflow.com/questions/3823921/convert-big-endian-to-little-endian-when-reading-from-a-binary-file
 *
 * @param reference to value to flip in place
 */
template <class T>
void SwapEndian(T *p)
{
    unsigned char *mem = reinterpret_cast<unsigned char*>(p);
    std::reverse(mem, mem + sizeof(T));
}

} // namespace elm

#endif // ELM_IO_BINARY_H_
