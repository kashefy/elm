#include "elm/core/inputname.h"

using namespace std;
using namespace elm;

InputName::InputName(const string &name)
    : name_(name)
{
}

InputName::operator std::string() const
{
    return name_;
}
