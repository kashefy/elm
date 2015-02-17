/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_PCL_POINT_TRAITS_H_
#define _ELM_CORE_PCL_POINT_TRAITS_H_

#ifdef __WITH_PCL // the following traits require PCL support

#include <pcl/common/io.h>      // getFields()

#include "elm/core/exception.h"

namespace  elm {

/**
 * @brief template class to for wrapping PCL point traits
 */
template <class TPoint>
class PCLPointTraits_
{
public:

    /** @brief get no. of fields of a pcl point type.
     * This does not account for SSE alignment or mixed types
     * @return field count
     */
    static size_t FieldCount()
    {
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields<TPoint>(fields);

//        for(size_t i=0; i<fields.size(); i++)
//        {
//            pcl::PCLPointField f = fields[i];
//            std::cout<<f.name<<",";
//        }
//        std::cout<<std::endl;

        return fields.size();
    }

    /** @brief Get no. of floats occupied by Point struct
     * @return no. of floats occupied by point struct
     */
    static size_t NbFloats()
    {
        ELM_THROW_BAD_DIMS_IF(sizeof(TPoint) % sizeof(float) != 0, "This point type has non-float components");
        return sizeof(TPoint) / sizeof(float);
    }
};

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_CORE_PCL_POINT_TRAITS_H_
