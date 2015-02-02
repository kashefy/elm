#ifndef _ELM_CORE_INPUTNAME_H_
#define _ELM_CORE_INPUTNAME_H_

#include <string>

#include <boost/property_tree/ptree.hpp>

namespace elm {

class InputName
{
public:
    InputName(const std::string &name);

    InputName(const boost::property_tree::ptree &p);
};

} // namespace elm

#endif // _ELM_CORE_INPUTNAME_H_
