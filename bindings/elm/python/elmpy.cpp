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
#include <boost/python/exception_translator.hpp>

// STL includes need to be included after boost on OS X
#include <vector>

#if !PYTHON_USE_NUMPY
#error "The module can only be built if NumPy is available"
#endif

#include "opencv2/core/core.hpp"

#include "elm/core/core.h"
#include "elm/core/exception.h"
#include "elm/core/typedefs_sfwd.h"

#include "elm/python/conversions.h"
#include "elm/python/conversions_cv.h"
#include "elm/python/signalpy.h"

namespace bp=boost::python;
using namespace cv;
using namespace elm;

#ifdef __GNUC__
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

template<>  void* type_from_python<Mat>::convertible( PyObject* obj )
{
    return PyArray_Check( obj ) ? obj : 0;
}

template<> void type_from_python<Mat>::construct( PyObject* obj, bp::converter::rvalue_from_python_stage1_data* data )
{
    Mat m;
    ArgInfo info("mat", false);

    if(!pyopencv_to(obj, m, info)) {

        std::string msg("Failed to convert array to Mat");
        PyErr_SetString(PyExc_ValueError, msg.c_str());
        bp::throw_error_already_set();
    }

    // Grab pointer to memory into which to construct the new C++ object
    void* storage = ((bp::converter::rvalue_from_python_storage<Mat >*) data)->storage.bytes;

    // in-place construct the new target C++ object using the character data extracted from the python object
    new (storage) Mat(m);

    // Stash the memory chunk pointer for later use by boost.python
    data->convertible = storage;
}

/**
 * @brief cv::Mat to numpy.ndarray conversion. With shallow copy of data.
 * @param Mat object to convert
 * @return pointer to new PyObject
 */
template<> PyObject* type_into_python<Mat>::convert(Mat const& t) {

    if(!t.isContinuous()) {

        std::string msg("Error converting non-continuous Mat");
        PyErr_SetString(PyExc_ValueError, msg.c_str());
        bp::throw_error_already_set();
    }

    // Possible bug in opencv for getting size of n-dim Mat
    // see: http://stackoverflow.com/questions/18882242/how-do-i-get-the-size-of-a-multi-dimensional-cvmat-mat-or-matnd
    // will focus on 2-dim Mats for now
    if(t.dims == 2) {

        return pyopencv_from(t);
    }
    else {

        std::stringstream sstream;
        sstream << "Mat with (" << t.dims <<") dims != 2 not supported.";
        PyErr_SetString(PyExc_ValueError, sstream.str().c_str());
        bp::throw_error_already_set();
    }

    Py_RETURN_NONE;
}

template<>  void* type_from_python<Mat1f>::convertible( PyObject* obj )
{
    return PyArray_Check( obj ) ? obj : 0;
}

template<> void type_from_python<Mat1f>::construct( PyObject* obj, bp::converter::rvalue_from_python_stage1_data* data )
{
    Mat1f m;
    ArgInfo info("mat", false);

    if(!pyopencv_to(obj, m, info)) {

        std::string msg("Failed to convert array to Mat1f");
        PyErr_SetString(PyExc_ValueError, msg.c_str());
        bp::throw_error_already_set();
    }

    // Grab pointer to memory into which to construct the new C++ object
    void* storage = ((bp::converter::rvalue_from_python_storage<Mat1f >*) data)->storage.bytes;

    // in-place construct the new target C++ object using the character data extracted from the python object
    new (storage) Mat1f(m);

    // Stash the memory chunk pointer for later use by boost.python
    data->convertible = storage;
}

/**
 * @brief Mat1f to numpy.ndarray conversion (shallow copy)
 * @param Mat1f (single-channel Mat of floats) object to convert
 * @return pointer to new PyObject
 */
template<> PyObject* type_into_python<Mat1f>::convert(Mat1f const& t) {

    return type_into_python<Mat>::convert(static_cast<Mat1f>(t));
}

/**
 * @brief Dummy class for exposing bindings for test purposes
 */
class DummyPy
{
public:
    DummyPy(){;}

    /**
     * @brief invokes numpy.ndarray to Mat conversion
     */
    void SetMat(const Mat& m)
    {
        m_ = m;
    }

    /**
     * @brief invokes Mat -> numpy.ndarray conversion
     * @return Mat initialized with mock values
     */
    Mat GetMat()
    {
        return m_;
    }

    /**
     * @brief invokes numpy.ndarray to Mat1f conversion
     */
    void SetMat1f(const Mat& m)
    {
        mat1f_ = static_cast<Mat1f>(m);
    }

    /**
     * @brief invokes Mat1f -> numpy.ndarray conversion
     * @return Mat1f initialized with mock values
     */
    Mat1f GetMat1f()
    {
        return mat1f_;
    }

protected:
    Mat1f mat1f_;
    Mat m_;
};

void SignalPy::fromPythonDict(const bp::dict &d) {

    bp::list keys = d.keys();
    const size_t NB_KEYS = bp::len(keys);

    for(size_t i=0; i<NB_KEYS; i++) {

        bp::object obj = keys[i];
        if(!PyString_Check(obj.ptr())) {

            PyErr_SetString(PyExc_TypeError, "Only keys of strings supported.");
            bp::throw_error_already_set();
        }
        else {

            string key = bp::extract<string >(bp::str(obj));
            bp::object obj_vals = d.get(key);
            if(PyList_Check(obj_vals.ptr())) {

                const bp::list vals = bp::extract<bp::list >(obj_vals);
                const size_t NB_ELEMS = bp::len(vals);

                for(size_t j=0; j<NB_ELEMS; j++) {

                    bp::object obj_elem = vals[j];
                    if(PyArray_Check(obj_elem.ptr())) {

                        Mat1f m;
                        ArgInfo info("mat", false);
                        if(!pyopencv_to(obj_elem.ptr(), m, info)) {

                            string msg("Failed to convert array to Mat");
                            PyErr_SetString(PyExc_ValueError, msg.c_str());
                            bp::throw_error_already_set();
                        }

                        Append(key, m);
                    }
                    else {

                        PyErr_SetString(PyExc_TypeError,
                                        "Only values of ndarray supported.");
                        bp::throw_error_already_set();
                    }
                }
            }
            else {

                PyErr_SetString(PyExc_TypeError,
                                "Unsupported value types");
                bp::throw_error_already_set();
            }
        }
    }
}

void translate_exception(ExceptionKeyError const& e) {

    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_KeyError, e.what());
}

// select overloaded method
void (SignalPy::*AppendMat1f)(const std::string&, const Mat1f&) = &SignalPy::Append;

BOOST_PYTHON_MODULE(elm) {

    // import additional python modules
    import_array();
    bp::numeric::array::set_module_and_type("numpy", "ndarray");

    // register custom converters
    bp::to_python_converter< VecS, type_into_python<VecS > >();

    type_from_python< Mat >();
    bp::to_python_converter< Mat, type_into_python<Mat > >();

    type_from_python< Mat1f >();
    bp::to_python_converter< Mat1f, type_into_python<Mat1f > >();

    // exceptions
    bp::register_exception_translator<ExceptionKeyError >(&translate_exception);

    // define top-level attributes
    bp::scope().attr("__version__") = GetVersion();

    bp::class_<DummyPy>("Dummy")
            .def("setMat",   &DummyPy::SetMat    )
            .def("getMat",   &DummyPy::GetMat    )
            .def("setMat1f", &DummyPy::SetMat1f  )
            .def("getMat1f", &DummyPy::GetMat1f  )
            ;

    bp::class_<SignalPy>("Signal")
            .def("append",          AppendMat1f                 )
            .def("clear",           &SignalPy::Clear            )
            .def("feature_names",   &SignalPy::FeatureNames     )
            .def("most_recent_mat1f", &SignalPy::MostRecentMat1f)
            .def("to_dict",         &SignalPy::toPythonDict     )
            .def("from_dict",       &SignalPy::fromPythonDict   )
            ;
}
