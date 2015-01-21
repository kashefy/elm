#ifndef SEM_CORE_PCL_POINT_TRAITS_H_
#define SEM_CORE_PCL_POINT_TRAITS_H_

#ifdef __WITH_PCL // the following traits require PCL support

#include <pcl/common/io.h>      // getFields()

namespace  sem {

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
        static_assert(sizeof(TPoint) % sizeof(float) == 0, "This point type has non-float components");
        return sizeof(TPoint) / sizeof(float);
    }
};

} // namespace sem

#else // __WITH_PCL
    #warning "Unable to define additional point_traits without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_PCL_POINT_TRAITS_H_
