/*
 * Sinc our module is initialized elsewhere and we call import_array() there,
 * we need to avoid over-importing the array module
 * according to these instructions: http://docs.scipy.org/doc/numpy/reference/c-api.array.html#importing-the-api
 */
#define PY_ARRAY_UNIQUE_SYMBOL COOL_ARRAY_API
#define NO_IMPORT_ARRAY
#include "elm/python/numpyallocator.h"

NumpyAllocator::~NumpyAllocator()
{
}

NumpyAllocator::NumpyAllocator()
{
}

void NumpyAllocator::allocate(int dims, const int* sizes, int type, int*& refcount,
              uchar*& datastart, uchar*& data, size_t* step)
{
    PyEnsureGIL gil;

    int depth = CV_MAT_DEPTH(type);
    int cn = CV_MAT_CN(type);
    const int f = (int)(sizeof(size_t)/8);
    int typenum = depth == CV_8U ? NPY_UBYTE : depth == CV_8S ? NPY_BYTE :
                  depth == CV_16U ? NPY_USHORT : depth == CV_16S ? NPY_SHORT :
                  depth == CV_32S ? NPY_INT : depth == CV_32F ? NPY_FLOAT :
                  depth == CV_64F ? NPY_DOUBLE : f*NPY_ULONGLONG + (f^1)*NPY_UINT;
    int i;
    npy_intp _sizes[CV_MAX_DIM+1];
    for( i = 0; i < dims; i++ )
        _sizes[i] = sizes[i];
    if( cn > 1 )
    {
        /*if( _sizes[dims-1] == 1 )
            _sizes[dims-1] = cn;
        else*/
            _sizes[dims++] = cn;
    }
    PyObject* o = PyArray_SimpleNew(dims, _sizes, typenum);
    if(!o)
        CV_Error_(CV_StsError, ("The numpy array of typenum=%d, ndims=%d can not be created", typenum, dims));
    refcount = refcountFromPyObject(o);
    npy_intp* _strides = PyArray_STRIDES((PyArrayObject*) o);
    for( i = 0; i < dims - (cn > 1); i++ )
        step[i] = (size_t)_strides[i];
    datastart = data = (uchar*)PyArray_DATA((PyArrayObject*) o);
}

void NumpyAllocator::deallocate(int* refcount, uchar*, uchar*)
{
    PyEnsureGIL gil;
    if( !refcount )
        return;
    PyObject* o = pyObjectFromRefcount(refcount);
    Py_INCREF(o);
    Py_DECREF(o);
}
