/** @file conversion functions to serve python bindings.
 *  Several adopted from OpneCV.
 */
#ifndef _ELM_PYTHON_CONVERSIONS_H_
#define _ELM_PYTHON_CONVERSIONS_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1800)
#define HAVE_ROUND  // eliminating duplicated round() declaration
#endif

#include <boost/python.hpp>

#if !PYTHON_USE_NUMPY
#error "The module can only be built if NumPy is available"
#endif

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

/**
 *  Conversion template
 */
template< typename T >
struct type_into_python
{
    static PyObject* convert( T const& );
};

template< typename T >
struct type_from_python
{
    type_from_python()
    {
        boost::python::converter::registry::push_back( convertible,
                                                       construct,
                                                       boost::python::type_id<T>() );
    }

    static void* convertible( PyObject* );
    static void construct( PyObject*, boost::python::converter::rvalue_from_python_stage1_data* );
};

#endif // _ELM_PYTHON_CONVERSIONS_H_
