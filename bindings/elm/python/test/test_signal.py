from nose.tools import assert_is_instance, assert_is_not_none, assert_list_equal, assert_true
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
        
    def test_feature_names(self):
        
        names = ["foo", "bar", "x"]
        acc = []
        
        x = np.random.rand(3, 2)
        
        for n in names:
            
            acc.append(n)
            self._to.append(n, x)
            assert_list_equal(sorted(self._to.feature_names()), sorted(acc))
        
        