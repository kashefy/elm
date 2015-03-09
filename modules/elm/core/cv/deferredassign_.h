#ifndef _ELM_CORE_CV_DEFERREDASSIGN__H_
#define _ELM_CORE_CV_DEFERREDASSIGN__H_

#include <boost/tuple/tuple.hpp>

#include <opencv2/core/core.hpp>

namespace elm {

/**
 * @brief class for deferring Mat element assignment operations
 */
template<class TElem>
class DeferredAssign_
{
public:
    DeferredAssign_()
    {
    }

    /**
     * @brief push_back record for deferred assignment/substitution
     * @param src current value
     * @param dst desired value
     */
    void push_back(TElem src, TElem dst)
    {
        subs_.push_back(std::make_pair(src, dst));
    }

    /**
     * @brief clear substitution records
     */
    void clear()
    {
        subs_.clear();
    }

    /**
     * @brief perform deferred assignment and clear substition records
     * @param m matrix to perform assignments on (in-place)
     */
    void assign(cv::Mat_<TElem > &m)
    {
        if(!m.empty()) {

            backwards();
            forwards(m);
            clear();
        }
    }

protected:
    typedef std::pair<TElem, TElem> Substitution;   ///< typedef for substitution

    /**
     * @brief traverse backwards in substitution list
     * to short-circuit intermediate substitutions
     */
    void backwards()
    {
        const int nb_subs = static_cast<int>(subs_.size());

        for(int i=nb_subs-1; i>=0; i--) {

            float src_i, dst_i;
            boost::tie(src_i, dst_i) = subs_[i];

            if(src_i != dst_i) {

                for(int j=i-1; j>=0; j--) {

                    float src_j, dst_j;
                    boost::tie(src_j, dst_j) = subs_[j];

                    if(src_i == dst_j) {

                        subs_[j].second = dst_i;
                    }
                }
            }
        }
    }

    /**
     * @brief forward traversal by OR-ing masks with common destination value
     */
    void forwards(cv::Mat_<TElem > &m)
    {
        for(size_t i=0; i<subs_.size(); i++) {

            float src_i, dst_i;
            boost::tie(src_i, dst_i) = subs_[i];

            if(src_i != dst_i) {

                cv::Mat mask = m == src_i;

                for(size_t j=i+1; j<subs_.size(); j++) {

                    float src_j, dst_j;
                    boost::tie(src_j, dst_j) = subs_[j];

                    if(dst_j == dst_i) {

                        cv::Mat mask_j = m == src_j;

                        if(countNonZero(mask_j) > 0) {

                            cv::bitwise_or(mask, mask_j, mask, mask_j);
                        }

                        subs_[j].second = src_j;
                    }
                }

                if(countNonZero(mask) > 0) {

                    m.setTo(dst_i, mask); // overwrite values with OR'ed mask
                }
            }
        }
    }

    // members
    std::vector<Substitution > subs_; ///< record substitutions

};

} // namespace elm

#endif // _ELM_CORE_CV_DEFERREDASSIGN__H_
