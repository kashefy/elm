#ifndef SEM_CORE_VISITOR__H_
#define SEM_CORE_VISITOR__H_

#include <boost/variant/static_visitor.hpp>

/**
 * @brief base class for caching heavy data type conversions
 */
class base_ConversionCache
{
public:
    virtual ~base_ConversionCache();

    /**
     * @brief Reset cache indicating a conversion took place
     */
    virtual void Reset();
protected:
    /**
     * @brief keep constructor protected, treat this as an interface
     */
    base_ConversionCache();
};

/**
 * @brief template class for different static visitors
 */
template <typename T>
class Visitor_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
{
};

/**
 * @brief A do-nothing visitor class to use in place of unsupported visitors
 */
class VisitorVoid :
        public Visitor_<void>
{
};

#endif // SEM_CORE_VISITOR__H_
