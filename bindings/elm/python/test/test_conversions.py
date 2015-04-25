from nose.tools import assert_greater, assert_equal, assert_is_instance, assert_true, assert_tuple_equal
import numpy as np
import elm as elm

def test_Mat1f_ndarray_2d_dims():
	
	for rows in range(1, 11):
		for cols in range(1, 11):
			
			x = np.random.rand(rows, cols)*100.
			test_dummy = elm.Dummy()
			test_dummy.setMat(x)
			y = test_dummy.getMat1f()
			
			assert_equal(x.ndim, 2)
			assert_tuple_equal(x.shape, (rows, cols))
			
			assert_is_instance(y, np.ndarray)
			assert_greater(y.size, 0)
			assert_equal(y.shape, x.shape)
			assert_true(np.all(x==y))
			x += 10
			assert_true(np.all(x==y))
			assert_true(x is y)

def test_Mat1f_ndarray_2d_values():
        
    x = np.random.rand(3, 2)*100.
    test_dummy = elm.Dummy()
    test_dummy.setMat(x)
    y = test_dummy.getMat1f()
    
    assert_is_instance(y, np.ndarray)
    assert_greater(y.size, 0)
    assert_equal(y.shape, x.shape)
    assert_true(np.all(x==y))
    x += 10
    assert_true(np.all(x==y))
    assert_true(x is y)

def test_Mat_ndarray_2d_dims():
	
	for rows in range(1, 11):
		for cols in range(1, 11):
			
			x = np.random.rand(rows, cols)*100.
			test_dummy = elm.Dummy()
			test_dummy.setMat(x)
			y = test_dummy.getMat()
			
			assert_equal(x.ndim, 2)
			assert_tuple_equal(x.shape, (rows, cols))
			
			assert_is_instance(y, np.ndarray)
			assert_greater(y.size, 0)
			assert_equal(y.shape, x.shape)
			assert_true(np.all(x==y))
			x += 10
			assert_true(np.all(x==y))
			assert_true(x is y)
			
def Mat_ndarray_2d_values_floats():
    
    x = np.random.rand(3, 2)*100.
    test_dummy = elm.Dummy()
    test_dummy.setMat(x)
    y = test_dummy.getMat()
    
    assert_true(x.dtype.name.startswith('float'))
    assert_equal(x.dtype, y.dtype)
    
    assert_is_instance(y, np.ndarray)
    
    assert_greater(y.size, 0)
    assert_equal(y.shape, x.shape)
    assert_true(np.all(x==y))
    x += 10
    assert_true(np.all(x==y))
    assert_true(x is y)

def Mat_ndarray_2d_values_ints():
    
    x = np.random.rand(3, 2)*100.
    x = x.astype(int)
    
    test_dummy = elm.Dummy()
    test_dummy.setMat(x)
    y = test_dummy.getMat()
    
    assert_true(x.dtype.name.startswith('int'))
    assert_equal(x.dtype, y.dtype)
    
    assert_is_instance(y, np.ndarray)
    assert_greater(y.size, 0)
    assert_equal(y.shape, x.shape)
    assert_true(np.all(x==y))
    x += 10
    assert_true(np.all(x==y))
    assert_true(x is y)
