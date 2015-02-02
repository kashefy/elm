#ifndef _ELM_CORE_INPUTNAME_H_
#define _ELM_CORE_INPUTNAME_H_

#include <string>

namespace elm {

/**
 * @brief class for repsresenting layer input names
 */
class InputName
{
public:
    /**
     * @brief Construct InputName object from string
     * @param name of input in string representation
     */
    InputName(const std::string &name);

    /**
     * @brief Construct InputName object from c-style string
     * @param name of input in c-style string representation
     */
    InputName(const char *name);

    /**
     * @brief convert input name to STL string
     */
    std::string to_string() const;

    /**
     * @brief convert input name to STL string, using conversion operator
     */
    operator std::string() const;

    /**
     * @brief overload equality operator (case sensitive)
     * @param right-hand side to compare against
     * @return true on equal name
     */
    bool operator==(const InputName &rhs) const;

    /**
     * @brief overload inequality operator (case sensitive)
     * @param right-hand side to compar against
     * @return true on non-equal names
     */
    bool operator!=(const InputName &rhs) const;

protected:
    std::string name_;  ///< input name
};

} // namespace elm

// non-member case-sensitive comparison operator overloading
bool operator==(const std::string &lhs, const elm::InputName &rhs);

bool operator!=(const std::string &lhs, const elm::InputName &rhs);

#endif // _ELM_CORE_INPUTNAME_H_
