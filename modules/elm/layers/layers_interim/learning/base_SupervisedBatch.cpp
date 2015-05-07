/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/learning/base_SupervisedBatch.h"

#include "elm/core/exception.h"

using namespace elm;

const std::string base_SupervisedBatch::KEY_TRAIN_LABEL = detail::BASE_SUPERVISEDBATCH_KEY_TRAIN_LABEL;

base_SupervisedBatch::~base_SupervisedBatch()
{
}

base_SupervisedBatch::base_SupervisedBatch()
    : base_LearningLayer(),
      base_FeatureTransformationLayer()
{
}

void base_SupervisedBatch::Learn()
{
    ELM_THROW_NOT_IMPLEMENTED_WMSG("Batch learning requires all training data.");
}
