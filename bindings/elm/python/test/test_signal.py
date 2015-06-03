from nose.tools import assert_is_instance, assert_is_not_none, assert_true
import elm as elm

class TestSignalInit:
    
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
        
