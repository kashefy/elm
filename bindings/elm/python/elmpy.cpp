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

// STL includes need to be included after boost on OS X
#include <vector>

#if !PYTHON_USE_NUMPY
#error "The module can only be built if NumPy is available"
#endif

#include "opencv2/core/core.hpp"

#include "elm/core/core.h"

#include "elm/python/conversions.h"
#include "elm/python/conversions_cv.h"

namespace bp=boost::python;
using namespace cv;
using namespace elm;

#ifdef __GNUC__
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

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
     * @brief invokes numpy.ndarray to Mat1f conversion
     */
    void SetMat(const bp::numeric::array& arr)
    {
        mat1f_ = Mat1f();
        ArgInfo info("mat", false);
        if(!pyopencv_to(arr.ptr(), mat1f_, info)) {

            std::string msg("Failed to convert array to Mat1f");
            PyErr_SetString(PyExc_ValueError, msg.c_str());
            bp::throw_error_already_set();
        }
    }

    /**
     * @brief invokes Mat1f -> numpy.ndarray conversion
     * @return Mat1f initialized with mock values
     */
    Mat1f GetMat1f()
    {
        return mat1f_;
    }

    /**
     * @brief invokes Mat1f -> numpy.ndarray conversion
     * @return Mat1f initialized with mock values
     */
    Mat GetMat()
    {
        return static_cast<Mat>(mat1f_);
    }

protected:
    Mat1f mat1f_;
};

BOOST_PYTHON_MODULE(elm) {

    // import additional python modules
    import_array();
    bp::numeric::array::set_module_and_type("numpy", "ndarray");

    // register custom converters
    bp::to_python_converter< Mat, type_into_python<Mat> >();
    bp::to_python_converter< Mat1f, type_into_python<Mat1f> >();

    // define top-level attributes
    bp::scope().attr("__version__") = GetVersion();

    bp::class_<DummyPy>("Dummy")
            .def("setMat",   &DummyPy::SetMat    )
            .def("getMat1f", &DummyPy::GetMat1f  )
            .def("getMat",   &DummyPy::GetMat    )
            ;
}
