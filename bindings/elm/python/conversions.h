/** @file conversion functions to serve python bindings.
 *  Several adopted from OpneCV.
 */
#ifndef _ELM_PYTHON_CONVERSIONS_H_
#define _ELM_PYTHON_CONVERSIONS_H_

#include <boost/python.hpp>

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
