#ifndef _ELM_CORE_INPUTNAME_H_
#define _ELM_CORE_INPUTNAME_H_

#include <string>

namespace elm {

class InputName
{
public:
    InputName(const std::string &name);

    operator std::string() const;

protected:
    std::string name_;  ///< input name
};

} // namespace elm

#endif // _ELM_CORE_INPUTNAME_H_
