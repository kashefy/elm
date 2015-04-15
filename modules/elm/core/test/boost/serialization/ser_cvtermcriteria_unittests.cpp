/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/serialization/ser_cvtermcriteria.h"

#include "elm/ts/serialization_assertions.h"


static CvTermCriteria CreateExampleCvTermCriteria()
{
	CvTermCriteria c;

	c.epsilon = 0.01;
	c.max_iter = 100;
	c.type = CV_TERMCRIT_EPS;

	return c;
}

template<> CvTermCriteria SerializationTypeAttr_<CvTermCriteria>::example = CreateExampleCvTermCriteria();

namespace {


INSTANTIATE_TYPED_TEST_CASE_P(SerializationTypedTest_CvTermCriteria_Test, SerializationTypedTest, CvTermCriteria);

} // annonymous namespace for unit tests
