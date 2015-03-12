#ifndef _ELM_CORE_CV_DEFERREDASSIGNHASHED__H_
#define _ELM_CORE_CV_DEFERREDASSIGNHASHED__H_

#include <unordered_map>
#include <unordered_set>

#include "elm/core/cv/deferredassign_.h"

namespace elm {

/**
 * @brief class for deferring Mat element assignment operations
 */
template<class TElem>
class DeferredAssignHashed_ : public DeferredAssign_<TElem >
{
public:
    virtual ~DeferredAssignHashed_()
    {
    }

    DeferredAssignHashed_()
        : DeferredAssign_<TElem >()
    {
    }

protected:

    /**
     * @brief forward traversal by OR-ing masks with common destination value
     */
    void forwards(cv::Mat_<TElem > &m)
    {
        std::unordered_map<TElem, std::unordered_set<TElem> > subs_map;

        const int nb_subs = static_cast<int>(this->subs_.size());
        for(int i=0; i<nb_subs; i++) {

            TElem src, dst;
            boost::tie(src, dst) = this->subs_[i];
            subs_map[dst].insert(src);
        }

        for (auto it = subs_map.begin(); it != subs_map.end(); ++it ) {

            float dst = it->first;

            cv::Mat mask;

            for (const auto& src: it->second) {

                /* ... process elem ... */
                if (mask.empty()) {

                     mask = m == src;
                }
                else {

                    cv::Mat mask_i = m == src;
                    cv::bitwise_or(mask, mask_i, mask, mask_i);
                }
            }

            if(countNonZero(mask) > 0) {

                m.setTo(dst, mask); // overwrite values with OR'ed mask
            }
        }
    }

    // members

};

} // namespace elm

#endif // _ELM_CORE_CV_DEFERREDASSIGNHASHED__H_
