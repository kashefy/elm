from nose.tools import assert_equals, assert_greater, assert_is_instance, assert_is_not_none, assert_list_equal, assert_true, assert_false, assert_raises
import numpy as np
import elm as elm

class TestSignalInst:
    
    def setup(self):
        
        self._to = elm.Signal()
   
    def teardown(self):
        
        self._to = None
        
    def test_instantiated(self):
        
        assert_is_not_none(self._to)
        
    def test_instance_type_name(self):
        
        assert_true("Signal" in str(type(self._to)))
        
    def test_instance_type(self):
        
        assert_is_instance(self._to, elm.Signal)
        
class TestSignal:
    
    def setup(self):
        
        self._to = elm.Signal()
   
    def teardown(self):
        
        self._to.clear()
        self._to = None
        
    def test_feature_names_empty(self):
        
        names = self._to.feature_names()
        assert_is_instance(names, list)
        assert_equals(len(names), 0)
        
    def test_feature_names(self):
        
        names = ["foo", "bar", "x"]
        acc = []
        
        x = np.random.rand(3, 2)
        
        for n in names:
            
            acc.append(n)
            self._to.append(n, x)
            assert_list_equal(sorted(self._to.feature_names()), sorted(acc))
            
    def test_append(self):
                
        foo = np.arange(1,5, dtype='float32').reshape((2, 2))
        bar = np.arange(1,5, dtype='float32').reshape((2, 2))+10
        
        self._to.append('foo', foo)
        self._to.append('bar', bar)
        
        foo2 = self._to.most_recent_mat1f("foo")
        
        assert_equals(foo2.shape, foo.shape)
        assert_true(np.all(foo2==foo))
        
        bar2 = self._to.most_recent_mat1f("bar")
        
        assert_equals(bar2.shape, bar.shape)
        assert_true(np.all(bar2==bar))
        
        assert_false(np.any(bar==foo))
        
        foo += 100
        assert_true(np.all(foo2==foo))
        
    def test_clear(self):
                
        foo = np.arange(1,5, dtype='float32').reshape((2, 2))
        bar = np.arange(1,5, dtype='float32').reshape((2, 2))+10
        
        self._to.append('foo', foo)
        self._to.append('bar', bar)
        
        assert_greater(len(self._to.feature_names()), 0)
        assert_is_not_none(self._to.most_recent_mat1f("foo"))
        assert_is_not_none(self._to.most_recent_mat1f("bar"))
        
        self._to.clear()
        
        assert_equals(len(self._to.feature_names()), 0)
        assert_raises(KeyError, self._to.most_recent_mat1f, "foo")
        assert_raises(KeyError, self._to.most_recent_mat1f, "bar")
        
        