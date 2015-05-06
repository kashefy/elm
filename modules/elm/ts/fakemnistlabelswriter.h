#ifndef _ELM_TS_FAKEMNISTLABELSWRITER_H_
#define _ELM_TS_FAKEMNISTLABELSWRITER_H_

#include <fstream>

#include <boost/filesystem.hpp>

namespace elm {

class FakeMNISTLabelsWriter {

public:

    static const int NB_ITEMS = 10;

    virtual ~FakeMNISTLabelsWriter();

    FakeMNISTLabelsWriter(const boost::filesystem::path& p);

    /**
     * @brief Save fake data to file
     * magic number
     * total no. of items
     * series label values [0,10)
     */
    virtual void Save(int magic_number);

    void WriteInt(int n);

protected:

    virtual void SaveHeader(int magic_number);

    virtual void SaveItems();

    // members
    boost::filesystem::path path_;
    std::ofstream out_;
};

} // namespace elm

#endif // _ELM_TS_FAKEMNISTLABELSWRITER_H_
