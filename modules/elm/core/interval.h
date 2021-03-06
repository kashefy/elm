/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_INTERVAL_H_
#define _ELM_CORE_INTERVAL_H_

/**
 * @brief Base class for intervals of real numbers
 */
class base_Interval {

public:
    virtual ~base_Interval();

    /**
     * @brief Check whether value is part of interval
     * @param x value to check
     * @return true if value is part of interval
     */
    virtual bool In(float x) const = 0;

protected:
    /**
     * @brief Constructor
     * @param a left endpoint
     * @param b right endpoint
     */
    base_Interval(float a, float b);

    float a_;   ///< left endpoint
    float b_;   ///< right endpoint
};

/**
 * @brief Closed interval
 * \see base_Interval
 */
class IntervalClosed : public base_Interval
{
public:
    IntervalClosed(float a, float b);

    /**
     * @brief x in [a, b]?
     * @param x value to check
     * @return true on yes
     */
    bool In(float x) const;
};

/**
 * @brief Left-Closed Right-Open interval
 * \see base_Interval
 */
class IntervalLClosedROpen : public base_Interval
{
public:
    IntervalLClosedROpen(float a, float b);

    /**
     * @brief x in [a, b)?
     * @param x value to check
     * @return true on yes
     */
    bool In(float x) const;
};

#endif // _ELM_CORE_INTERVAL_H_
