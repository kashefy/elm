#include "elm/core/inputname.h"

using namespace std;
using namespace elm;

InputName::InputName(const string &name)
    : name_(name)
{
}

InputName::InputName(const char* name)
    : name_(name)
{
}

std::string InputName::to_string() const
{
    return name_;
}

InputName::operator std::string() const
{
    return to_string();
}

bool InputName::operator==(const InputName &rhs) const
{
    /* do actual comparison */
    return to_string() == rhs.to_string();
}

bool InputName::operator!=(const InputName &rhs) const
{
    return !this->operator==(rhs);
}

// define comparison operators
bool operator==(const std::string &lhs, const elm::InputName &rhs)
{
    return rhs == lhs;
}

bool operator!=(const std::string &lhs, const elm::InputName &rhs)
{
    return !(lhs == rhs);
}
