#ifndef _ELM_PYTHON_ARGINFO_H_
#define _ELM_PYTHON_ARGINFO_H_

/**
 * @brief Struct for holding argument info.
 * Originally from OpenCV python bindings.
 *
 * more fields may be added if necessary
 */
struct ArgInfo
{
    const char *name;
    bool outputarg;

    ArgInfo(const char * name_, bool outputarg_)
        : name(name_)
        , outputarg(outputarg_) {}

    /**
     * @brief get name
     * to match with older pyopencv_to function signature
     * @return nameto match with older pyopencv_to function signature
     */
    operator const char *() const { return name; }
};

#endif // _ELM_PYTHON_ARGINFO_H_
