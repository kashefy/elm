/** @file This is where we initialize our python module
 *
 * Since this is where the initialization happens, import_array() belongs here
 * We need to avoid over-importing the array module
 * according to these instructions: http://docs.scipy.org/doc/numpy/reference/c-api.array.html#importing-the-api
 */
#define PY_ARRAY_UNIQUE_SYMBOL COOL_ARRAY_API

#if defined(_MSC_VER) && (_MSC_VER >= 1800)
#define HAVE_ROUND  // eliminating duplicated round() declaration
#endif

#include <boost/python.hpp>

#include "elm/core/core.h"

#if !PYTHON_USE_NUMPY
#error "The module can only be built if NumPy is available"
#endif

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

namespace bp=boost::python;
using namespace elm;

BOOST_PYTHON_MODULE(elm) {

    // import additional python modules
    import_array();
    bp::numeric::array::set_module_and_type("numpy", "ndarray");

    // define top-level attributes
    bp::scope().attr("__version__") = GetVersion();
}
