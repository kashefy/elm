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

<<<<<<< HEAD
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
=======
    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF()) = 0;

>>>>>>> b44d1cfcbe7fd993169839faa82cf1f7fb19c105
    virtual MatF PopCode() = 0;

protected:
    base_PopulationCode();
};

<<<<<<< HEAD
/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 */
=======
>>>>>>> b44d1cfcbe7fd993169839faa82cf1f7fb19c105
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    virtual void State(const MatF& in, const VecMatF& kernels=VecMatF());

    virtual MatF PopCode();

protected:
<<<<<<< HEAD
    MatF state_;    ///< internal state
=======
    MatF state_;
>>>>>>> b44d1cfcbe7fd993169839faa82cf1f7fb19c105
};

#endif // SEM_ENCODING_POPULATIONCODE_H_
