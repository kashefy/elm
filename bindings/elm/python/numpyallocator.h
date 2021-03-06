#ifndef _ELM_PYTHON_NUMPYALLOCATOR_H_
#define _ELM_PYTHON_NUMPYALLOCATOR_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1800)
#define HAVE_ROUND  // eliminating duplicated round() declaration
#endif

#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include <opencv2/core/core.hpp>

#include "elm/python/arginfo.h"

class PyAllowThreads
{
public:
    PyAllowThreads() : _state(PyEval_SaveThread()) {}
    ~PyAllowThreads()
    {
        PyEval_RestoreThread(_state);
    }
private:
    PyThreadState* _state;
};

class PyEnsureGIL
{
public:
    PyEnsureGIL() : _state(PyGILState_Ensure()) {}
    ~PyEnsureGIL()
    {
        PyGILState_Release(_state);
    }
private:
    PyGILState_STATE _state;
};

#define ERRWRAP2(expr) \
try \
{ \
    PyAllowThreads allowThreads; \
    expr; \
} \
catch (const cv::Exception &e) \
{ \
    PyErr_SetString(opencv_error, e.what()); \
    return 0; \
}

static size_t REFCOUNT_OFFSET = (size_t)&(((PyObject*)0)->ob_refcnt) +
    (0x12345678 != *(const size_t*)"\x78\x56\x34\x12\0\0\0\0\0")*sizeof(int);

static inline PyObject* pyObjectFromRefcount(const int* refcount)
{
    return (PyObject*)((size_t)refcount - REFCOUNT_OFFSET);
}

static inline int* refcountFromPyObject(const PyObject* obj)
{
    return (int*)((size_t)obj + REFCOUNT_OFFSET);
}

/**
 * @brief OpenCV python binding's NumpyAllocator class
 */
class NumpyAllocator : public cv::MatAllocator
{
public:
    NumpyAllocator();
    ~NumpyAllocator();

    void allocate(int dims, const int* sizes, int type, int*& refcount,
                  uchar*& datastart, uchar*& data, size_t* step);

    void deallocate(int* refcount, uchar*, uchar*);
};

#endif // _ELM_PYTHON_NUMPYALLOCATOR_H_
