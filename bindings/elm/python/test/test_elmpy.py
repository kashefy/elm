from nose.tools import assert_greater, assert_is_instance, assert_is_not_none
import elm as elm

def test_version():
	assert_is_not_none(elm.__version__)
	assert_is_instance(elm.__version__, str)
	assert_greater(len(elm.__version__), 0)
