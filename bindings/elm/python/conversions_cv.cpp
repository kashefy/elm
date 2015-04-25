#include "elm/python/numpyallocator.h"

using namespace cv;

static PyObject* opencv_error = 0;

static int failmsg(const char *fmt, ...) {

    char str[1000];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(str, sizeof(str), fmt, ap);
    va_end(ap);

    PyErr_SetString(PyExc_TypeError, str);
    return 0;
}

NumpyAllocator g_numpyAllocator;

enum {
    ARG_NONE = 0,
    ARG_MAT = 1,
    ARG_SCALAR = 2
};

// special case, when the convertor needs full ArgInfo structure
static int pyopencv_to(const PyObject* o, Mat& m, const ArgInfo info, bool allowND=true)
{
    if(!o || o == Py_None)
    {
        if( !m.data )
            m.allocator = &g_numpyAllocator;
        return true;
    }

    if( PyInt_Check(o) )
    {
        double tmp = static_cast<double>(PyInt_AsLong((PyObject*)o));
        double v[] = {tmp, 0., 0., 0.};
        m = Mat(4, 1, CV_64F, v).clone();
        return true;
    }
    if( PyFloat_Check(o) )
    {
        double v[] = {PyFloat_AsDouble((PyObject*)o), 0., 0., 0.};
        m = Mat(4, 1, CV_64F, v).clone();
        return true;
    }
    if( PyTuple_Check(o) )
    {
        int i, sz = (int)PyTuple_Size((PyObject*)o);
        m = Mat(sz, 1, CV_64F);
        for( i = 0; i < sz; i++ )
        {
            PyObject* oi = PyTuple_GET_ITEM(o, i);
            if( PyInt_Check(oi) )
                m.at<double>(i) = (double)PyInt_AsLong(oi);
            else if( PyFloat_Check(oi) )
                m.at<double>(i) = (double)PyFloat_AsDouble(oi);
            else
            {
                failmsg("%s is not a numerical tuple", info.name);
                m.release();
                return false;
            }
        }
        return true;
    }

    if( !PyArray_Check(o) )
    {
        failmsg("%s is not a numpy array, neither a scalar", info.name);
        return false;
    }

    PyArrayObject* oarr = (PyArrayObject*) o;

    bool needcopy = false, needcast = false;
    int typenum = PyArray_TYPE(oarr), new_typenum = typenum;
    int type = typenum == NPY_UBYTE ? CV_8U :
               typenum == NPY_BYTE ? CV_8S :
               typenum == NPY_USHORT ? CV_16U :
               typenum == NPY_SHORT ? CV_16S :
               typenum == NPY_INT ? CV_32S :
               typenum == NPY_INT32 ? CV_32S :
               typenum == NPY_FLOAT ? CV_32F :
               typenum == NPY_DOUBLE ? CV_64F : -1;

    if( type < 0 )
    {
        if( typenum == NPY_INT64 || typenum == NPY_UINT64 || type == NPY_LONG )
        {
            needcopy = needcast = true;
            new_typenum = NPY_INT;
            type = CV_32S;
        }
        else
        {
            failmsg("%s data type = %d is not supported", info.name, typenum);
            return false;
        }
    }

    int ndims = PyArray_NDIM(oarr);
    if(ndims >= CV_MAX_DIM)
    {
        failmsg("%s dimensionality (=%d) is too high", info.name, ndims);
        return false;
    }

    int size[CV_MAX_DIM+1];
    size_t step[CV_MAX_DIM+1], elemsize = CV_ELEM_SIZE1(type);
    const npy_intp* _sizes = PyArray_DIMS(oarr);
    const npy_intp* _strides = PyArray_STRIDES(oarr);
    bool ismultichannel = ndims == 3 && _sizes[2] <= CV_CN_MAX;

    for( int i = ndims-1; i >= 0 && !needcopy; i-- )
    {
        // these checks handle cases of
        //  a) multi-dimensional (ndims > 2) arrays, as well as simpler 1- and 2-dimensional cases
        //  b) transposed arrays, where _strides[] elements go in non-descending order
        //  c) flipped arrays, where some of _strides[] elements are negative
        if( (i == ndims-1 && (size_t)_strides[i] != elemsize) ||
            (i < ndims-1 && _strides[i] < _strides[i+1]) )
            needcopy = true;
    }

    if( ismultichannel && _strides[1] != (npy_intp)elemsize*_sizes[2] )
        needcopy = true;

    if (needcopy)
    {
        if (info.outputarg)
        {
            failmsg("Layout of the output array %s is incompatible with cv::Mat (step[ndims-1] != elemsize or step[1] != elemsize*nchannels)", info.name);
            return false;
        }

        if( needcast ) {
            o = PyArray_Cast(oarr, new_typenum);
            oarr = (PyArrayObject*) o;
        }
        else {
            oarr = PyArray_GETCONTIGUOUS(oarr);
            o = (PyObject*) oarr;
        }

        _strides = PyArray_STRIDES(oarr);
    }

    for(int i = 0; i < ndims; i++)
    {
        size[i] = (int)_sizes[i];
        step[i] = (size_t)_strides[i];
    }

    // handle degenerate case
    if( ndims == 0) {
        size[ndims] = 1;
        step[ndims] = elemsize;
        ndims++;
    }

    if( ismultichannel )
    {
        ndims--;
        type |= CV_MAKETYPE(0, size[2]);
    }

    if( ndims > 2 && !allowND )
    {
        failmsg("%s has more than 2 dimensions", info.name);
        return false;
    }

    m = Mat(ndims, size, type, PyArray_DATA(oarr), step);

    if( m.data )
    {
        m.refcount = refcountFromPyObject(o);
        if (!needcopy)
        {
            m.addref(); // protect the original numpy array from deallocation
                        // (since Mat destructor will decrement the reference counter)
        }
    };
    m.allocator = &g_numpyAllocator;

    return true;
}

static PyObject* pyopencv_from(const Mat& m)
{
    if( !m.data )
        Py_RETURN_NONE;
    Mat temp, *p = (Mat*)&m;
    if(!p->refcount || p->allocator != &g_numpyAllocator)
    {
        temp.allocator = &g_numpyAllocator;
        ERRWRAP2(m.copyTo(temp));
        p = &temp;
    }
    p->addref();
    return pyObjectFromRefcount(p->refcount);
}

static inline PyObject* pyopencv_from(const Scalar& src)
{
    return Py_BuildValue("(dddd)", src[0], src[1], src[2], src[3]);
}

static PyObject* pyopencv_from(const string& value)
{
    return PyString_FromString(value.empty() ? "" : value.c_str());
}

static bool pyopencv_to(PyObject* obj, string& value, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    char* str = PyString_AsString(obj);
    if(!str)
        return false;
    value = string(str);
    return true;
}

static inline bool pyopencv_to(PyObject* obj, Size& sz, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    return PyArg_ParseTuple(obj, "ii", &sz.width, &sz.height) > 0;
}

static inline PyObject* pyopencv_from(const Size& sz)
{
    return Py_BuildValue("(ii)", sz.width, sz.height);
}

static inline bool pyopencv_to(PyObject* obj, Rect& r, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    return PyArg_ParseTuple(obj, "iiii", &r.x, &r.y, &r.width, &r.height) > 0;
}

static inline PyObject* pyopencv_from(const Rect& r)
{
    return Py_BuildValue("(iiii)", r.x, r.y, r.width, r.height);
}

static inline bool pyopencv_to(PyObject* obj, Range& r, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    if(PyObject_Size(obj) == 0)
    {
        r = Range::all();
        return true;
    }
    return PyArg_ParseTuple(obj, "ii", &r.start, &r.end) > 0;
}

static inline PyObject* pyopencv_from(const Range& r)
{
    return Py_BuildValue("(ii)", r.start, r.end);
}

static inline bool pyopencv_to(PyObject* obj, CvSlice& r, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    if(PyObject_Size(obj) == 0)
    {
        r = CV_WHOLE_SEQ;
        return true;
    }
    return PyArg_ParseTuple(obj, "ii", &r.start_index, &r.end_index) > 0;
}

static inline PyObject* pyopencv_from(const CvSlice& r)
{
    return Py_BuildValue("(ii)", r.start_index, r.end_index);
}

static inline bool pyopencv_to(PyObject* obj, Point& p, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    if(!!PyComplex_CheckExact(obj))
    {
        Py_complex c = PyComplex_AsCComplex(obj);
        p.x = saturate_cast<int>(c.real);
        p.y = saturate_cast<int>(c.imag);
        return true;
    }
    return PyArg_ParseTuple(obj, "ii", &p.x, &p.y) > 0;
}

static inline bool pyopencv_to(PyObject* obj, Point2f& p, const char* name = "<unknown>")
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    if(!!PyComplex_CheckExact(obj))
    {
        Py_complex c = PyComplex_AsCComplex(obj);
        p.x = saturate_cast<float>(c.real);
        p.y = saturate_cast<float>(c.imag);
        return true;
    }
    return PyArg_ParseTuple(obj, "ff", &p.x, &p.y) > 0;
}

static inline PyObject* pyopencv_from(const Point& p)
{
    return Py_BuildValue("(ii)", p.x, p.y);
}

static inline PyObject* pyopencv_from(const Point2f& p)
{
    return Py_BuildValue("(dd)", p.x, p.y);
}

static inline bool pyopencv_to(PyObject* obj, Vec3d& v, const char* name = "<unknown>")
{
    (void)name;
    if(!obj)
        return true;
    return PyArg_ParseTuple(obj, "ddd", &v[0], &v[1], &v[2]) > 0;
}

static inline PyObject* pyopencv_from(const Vec3d& v)
{
    return Py_BuildValue("(ddd)", v[0], v[1], v[2]);
}

static inline PyObject* pyopencv_from(const Point2d& p)
{
    return Py_BuildValue("(dd)", p.x, p.y);
}

template<typename _Tp> struct pyopencvVecConverter
{
    static bool to(PyObject* obj, vector<_Tp>& value, const ArgInfo info)
    {
        typedef typename DataType<_Tp>::channel_type _Cp;
        if(!obj || obj == Py_None)
            return true;
        if (PyArray_Check(obj))
        {
            Mat m;
            pyopencv_to(obj, m, info);
            m.copyTo(value);
        }
        if (!PySequence_Check(obj))
            return false;
        PyObject *seq = PySequence_Fast(obj, info.name);
        if (seq == NULL)
            return false;
        int i, j, n = (int)PySequence_Fast_GET_SIZE(seq);
        value.resize(n);

        int type = DataType<_Tp>::type;
        int depth = CV_MAT_DEPTH(type), channels = CV_MAT_CN(type);
        PyObject** items = PySequence_Fast_ITEMS(seq);

        for( i = 0; i < n; i++ )
        {
            PyObject* item = items[i];
            PyObject* seq_i = 0;
            PyObject** items_i = &item;
            _Cp* data = (_Cp*)&value[i];

            if( channels == 2 && PyComplex_CheckExact(item) )
            {
                Py_complex c = PyComplex_AsCComplex(obj);
                data[0] = saturate_cast<_Cp>(c.real);
                data[1] = saturate_cast<_Cp>(c.imag);
                continue;
            }
            if( channels > 1 )
            {
                if( PyArray_Check(item))
                {
                    Mat src;
                    pyopencv_to(item, src, info);
                    if( src.dims != 2 || src.channels() != 1 ||
                       ((src.cols != 1 || src.rows != channels) &&
                        (src.cols != channels || src.rows != 1)))
                        break;
                    Mat dst(src.rows, src.cols, depth, data);
                    src.convertTo(dst, type);
                    if( dst.data != (uchar*)data )
                        break;
                    continue;
                }

                seq_i = PySequence_Fast(item, info.name);
                if( !seq_i || (int)PySequence_Fast_GET_SIZE(seq_i) != channels )
                {
                    Py_XDECREF(seq_i);
                    break;
                }
                items_i = PySequence_Fast_ITEMS(seq_i);
            }

            for( j = 0; j < channels; j++ )
            {
                PyObject* item_ij = items_i[j];
                if( PyInt_Check(item_ij))
                {
                    int v = (int)PyInt_AsLong(item_ij);
                    if( v == -1 && PyErr_Occurred() )
                        break;
                    data[j] = saturate_cast<_Cp>(v);
                }
                else if( PyLong_Check(item_ij))
                {
                    int v = (int)PyLong_AsLong(item_ij);
                    if( v == -1 && PyErr_Occurred() )
                        break;
                    data[j] = saturate_cast<_Cp>(v);
                }
                else if( PyFloat_Check(item_ij))
                {
                    double v = PyFloat_AsDouble(item_ij);
                    if( PyErr_Occurred() )
                        break;
                    data[j] = saturate_cast<_Cp>(v);
                }
                else
                    break;
            }
            Py_XDECREF(seq_i);
            if( j < channels )
                break;
        }
        Py_DECREF(seq);
        return i == n;
    }

    static PyObject* from(const vector<_Tp>& value)
    {
        if(value.empty())
            return PyTuple_New(0);
        Mat src((int)value.size(), DataType<_Tp>::channels, DataType<_Tp>::depth, (uchar*)&value[0]);
        return pyopencv_from(src);
    }
};


template<typename _Tp> static inline bool pyopencv_to(PyObject* obj, vector<_Tp>& value, const ArgInfo info)
{
    return pyopencvVecConverter<_Tp>::to(obj, value, info);
}

template<typename _Tp> static inline PyObject* pyopencv_from(const vector<_Tp>& value)
{
    return pyopencvVecConverter<_Tp>::from(value);
}

template<typename _Tp> static inline bool pyopencv_to_generic_vec(PyObject* obj, vector<_Tp>& value, const ArgInfo info)
{
    if(!obj || obj == Py_None)
       return true;
    if (!PySequence_Check(obj))
        return false;
    PyObject *seq = PySequence_Fast(obj, info.name);
    if (seq == NULL)
        return false;
    int i, n = (int)PySequence_Fast_GET_SIZE(seq);
    value.resize(n);

    PyObject** items = PySequence_Fast_ITEMS(seq);

    for( i = 0; i < n; i++ )
    {
        PyObject* item = items[i];
        if(!pyopencv_to(item, value[i], info))
            break;
    }
    Py_DECREF(seq);
    return i == n;
}

template<typename _Tp> static inline PyObject* pyopencv_from_generic_vec(const vector<_Tp>& value)
{
    int i, n = (int)value.size();
    PyObject* seq = PyList_New(n);
    for( i = 0; i < n; i++ )
    {
        PyObject* item = pyopencv_from(value[i]);
        if(!item)
            break;
        PyList_SET_ITEM(seq, i, item);
    }
    if( i < n )
    {
        Py_DECREF(seq);
        return 0;
    }
    return seq;
}


template<typename _Tp> struct pyopencvVecConverter<vector<_Tp> >
{
    static bool to(PyObject* obj, vector<vector<_Tp> >& value, const char* name="<unknown>")
    {
        return pyopencv_to_generic_vec(obj, value, name);
    }

    static PyObject* from(const vector<vector<_Tp> >& value)
    {
        return pyopencv_from_generic_vec(value);
    }
};

template<> struct pyopencvVecConverter<Mat>
{
    static bool to(PyObject* obj, vector<Mat>& value, const ArgInfo info)
    {
        return pyopencv_to_generic_vec(obj, value, info);
    }

    static PyObject* from(const vector<Mat>& value)
    {
        return pyopencv_from_generic_vec(value);
    }
};

template<> struct pyopencvVecConverter<string>
{
    static bool to(PyObject* obj, vector<string>& value, const ArgInfo info)
    {
        return pyopencv_to_generic_vec(obj, value, info);
    }

    static PyObject* from(const vector<string>& value)
    {
        return pyopencv_from_generic_vec(value);
    }
};

static inline bool pyopencv_to(PyObject *obj, CvTermCriteria& dst, const char *name="<unknown>")
{
    (void)name;
    if(!obj)
        return true;
    return PyArg_ParseTuple(obj, "iid", &dst.type, &dst.max_iter, &dst.epsilon) > 0;
}

static inline PyObject* pyopencv_from(const CvTermCriteria& src)
{
    return Py_BuildValue("(iid)", src.type, src.max_iter, src.epsilon);
}

static inline bool pyopencv_to(PyObject *obj, TermCriteria& dst, const char *name="<unknown>")
{
    (void)name;
    if(!obj)
        return true;
    return PyArg_ParseTuple(obj, "iid", &dst.type, &dst.maxCount, &dst.epsilon) > 0;
}

static inline PyObject* pyopencv_from(const TermCriteria& src)
{
    return Py_BuildValue("(iid)", src.type, src.maxCount, src.epsilon);
}

static inline bool pyopencv_to(PyObject *obj, RotatedRect& dst, const char *name="<unknown>")
{
    (void)name;
    if(!obj)
        return true;
    return PyArg_ParseTuple(obj, "(ff)(ff)f", &dst.center.x, &dst.center.y, &dst.size.width, &dst.size.height, &dst.angle) > 0;
}

static inline PyObject* pyopencv_from(const RotatedRect& src)
{
    return Py_BuildValue("((ff)(ff)f)", src.center.x, src.center.y, src.size.width, src.size.height, src.angle);
}

template <class T>
static bool pyopencv_to(PyObject *o, Ptr<T>& p, const char *name="<unknown>")
{
    p = new T();
    return pyopencv_to(o, *p, name);
}

#define MKTYPE2(NAME) pyopencv_##NAME##_specials(); if (!to_ok(&pyopencv_##NAME##_Type)) return
