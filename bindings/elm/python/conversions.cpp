/*
 * Sinc our module is initialized elsewhere and we call import_array() there,
 * we need to avoid over-importing the array module
 * according to these instructions: http://docs.scipy.org/doc/numpy/reference/c-api.array.html#importing-the-api
 */
#define PY_ARRAY_UNIQUE_SYMBOL COOL_ARRAY_API
#define NO_IMPORT_ARRAY
#include "elm/python/conversions.h"
