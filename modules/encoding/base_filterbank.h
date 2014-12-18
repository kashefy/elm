/** @file Define classes for encapsulating, applying filter banks
 * and different response iterators (per kernel, per element across kernels).
 */
#ifndef SEM_ENCODING_BASE_FILTERBANK_H_
#define SEM_ENCODING_BASE_FILTERBANK_H_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/exception.h"
#include "core/typedefs.h"

/* Define iterator traits */
template<typename T> class NumberIterTraits;
template<> class NumberIterTraits<void>
{
public:
    static int next(const std::vector<int> &numbers, size_t sequence)
    {
        SEM_THROW_NOT_IMPLEMENTED_WMSG("NumberIterTraits<void>::next should not be used.");
    }
};


class Traits_Normal;
template<> class NumberIterTraits<Traits_Normal>
{
public:
    static int next(const std::vector<int> &numbers, size_t sequence)
    {
        return numbers[sequence];
    }
};


class Traits_Cumulative;
template<> class NumberIterTraits<Traits_Cumulative>
{
public:
    static int next(const std::vector<int> &numbers, size_t sequence)
    {
        if (sequence < 0)
            return 0;

        int value = 0;

        for (size_t i=0; i <= sequence; i++)
            value += numbers[i];

        return value;
    }
};

/**
 * Template filter bank iterator class.
 * We want to be able to easily choose between different iterating mechanisms
 * \see http://blog.cppse.nl/cpp-multiple-iterators-with-traits
 */
template <typename T, typename Traits = NumberIterTraits<T> >
class NumberIter: public std::iterator< std::forward_iterator_tag, std::string >
{
public:

    // Constructors
    NumberIter(const std::vector<int> &numbers, size_t seq)
        : numbers_(numbers), sequence_(seq)
    {}

    // Copy constructor
    NumberIter(const NumberIter<void> &other)
    {
        numbers_ = other.numbers_;
        sequence_ = other.sequence_;
    }

    // Operators
    const int operator*() const
    {
        return Traits::next(numbers_, sequence_);
    }

    NumberIter & operator++(int)
    {
        sequence_++;
        return *this;
    }

    template <typename N>
    bool operator==(const NumberIter<N>& other)
    {
        return sequence_ == other.sequence_;
    }

    template <typename X>
    bool operator!=(const NumberIter<X>& other)
    {
        return !((*this) == other);
    }

private:

    std::vector<int> numbers_;
    size_t sequence_;

    friend class NumberIter<Traits_Normal>;
    friend class NumberIter<Traits_Cumulative>;
};

/**
 * @brief base class for filter banks
 *  The interface should aid in iterating through response
 *  by defining different iterators,
 *  whether kernel-by-kernel or element response acorss all kernels
 */
class base_FilterBank
{
public:
    /** Empty default destructor
     */
    ~base_FilterBank();

    /**
     * @brief Compute response to all kernels
     * @param stimulus
     * @return response per kernel
     */
    virtual VecMat1f Compute(cv::Mat1f stimulus) {};

    NumberIter<void> begin()
    {
        return NumberIter<void>(numbers_, 0);
    }

    NumberIter<void> end()
    {
        return NumberIter<void>(numbers_, numbers_.size());
    }

    typedef NumberIter<Traits_Normal> iterator;
    typedef NumberIter<Traits_Cumulative> cumulative_iterator;


    /**
     * @brief Empty default constructor, only accessible by child classes
     */
    base_FilterBank() {

        for(int i=1; i<=10; i++) {
            numbers_.push_back(i);
        }
    }

protected:

    VecMat1f kernels_;  ///< individual kernels
    VecMat1f response_; ///< response per kernel for most recent input
    std::vector<int> numbers_; ///< dummy
};

#endif // SEM_ENCODING_BASE_FILTERBANK_H_
