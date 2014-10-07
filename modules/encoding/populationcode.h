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

    /**
     * @brief Compute internal state
     * @param in input
     * @param kernels vector of kernels
     */
    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF()) = 0;

    /**
     * @brief get population code
     * @return population code
     */
    virtual MatF PopCode() = 0;

protected:
    base_PopulationCode();
};

/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 */
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF());

    virtual MatF PopCode();

protected:
    MatF state_;    ///< internal state
};

#endif // SEM_ENCODING_POPULATIONCODE_H_
