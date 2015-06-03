/*
 * Since our module is initialized elsewhere and we call import_array() there,
 * we need to avoid over-importing the array module
 * according to these instructions: http://docs.scipy.org/doc/numpy/reference/c-api.array.html#importing-the-api
 */
#define PY_ARRAY_UNIQUE_SYMBOL COOL_ARRAY_API
#define NO_IMPORT_ARRAY
#include "elm/python/conversions.h"

#include "elm/core/typedefs_sfwd.h"

#include "elm/python/stl_inl.h"

namespace bp=boost::python;
using namespace elm;

/**
 * @brief Convert VecS to python list
 * @param[in] t source vector
 * @return python list populated with source vector elements
 */
template<> PyObject* type_into_python<VecS>::convert(const VecS& t) {

    return bp::incref( toPythonList(t).ptr() );
}
