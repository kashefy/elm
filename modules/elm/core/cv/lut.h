/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_CV_LUT_H_
#define _ELM_CORE_CV_LUT_H_

#include <vector>

#include "elm/core/cv/typedefs_fwd.h"

extern template class std::vector<int>;

namespace elm {

/**
 * @brief LUT class since OpenCV's only supports LUT with 8-bit depth
 *
 * This basically hashes an integer value by index
 */
class LUT
{
public:
    virtual ~LUT();

    /**
     * @brief alternate constructor
     * calling Capacity() still required.
     */
    LUT();

    /**
     * @brief alternate constructor
     * The capacity should also equal the largest value you expect inserting into the LUT
     * @param capacity expected table capacity
     */
    LUT(int capacity);

    /**
     * @brief Set expected look-up table capacity
     * The capacity should also equal the largest value you expect inserting into the LUT
     * @param n capacity
     */
    void Capacity(int n);

    /**
     * @brief insert entry into look up table
     * @param lut_entry new entry
     */
    void insert(int lut_entry);

    /**
     * @brief update an entry
     * @param a entry value
     * @param b entry value
     * @return persistent value in look-up table min(a,b)
     * @todo speed up traversal using secondary lut or per entry max
     */
    int update(int a, int b);

    /**
     * @brief apply look-up table to matrix
     * @param m input matrix with elements re-assigned in-place
     */
    void apply(cv::Mat1i &m) const;

protected:

    // members
    std::vector<int> table_;    ///< underlying look-up table with index serving as hash
    int count_;                 ///< no. of inserted elements
    int max_entry_;             ///< keeps track of highest values entry inserted
};

} // namespace elm

#endif // _ELM_CORE_CV_LUT_H_
