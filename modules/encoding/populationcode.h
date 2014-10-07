#ifndef SEM_ENCODING_POPULATIONCODE_H_
#define SEM_ENCODING_POPULATIONCODE_H_

#include <core/typedefs.h>

/**
 * @brief The base class for population codes
 */
class base_PopulationCode
{
public:
    virtual ~base_PopulationCode() {}

    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF()) = 0;

    virtual MatF PopCode() = 0;

protected:
    base_PopulationCode();
};

class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF());

    virtual MatF PopCode();

protected:
    MatF state_;
};

#endif // SEM_ENCODING_POPULATIONCODE_H_
