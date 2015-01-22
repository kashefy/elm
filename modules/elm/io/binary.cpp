#include "elm/io/io.h"

#include <algorithm>

template <class T>
void SwapEndian(T *p)
{
    unsigned char *mem = reinterpret_cast<unsigned char*>(p);
    std::reverse(mem, mem + sizeof(T));
}
