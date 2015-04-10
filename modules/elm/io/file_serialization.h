/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_FILE_SERIALIZATION_H_
#define _ELM_IO_FILE_SERIALIZATION_H_

#include <fstream>

#include <boost/filesystem.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include "elm/core/exception.h"

namespace elm {

namespace detail {

enum ArchiveType
{
    BIN,
    TXT,
    XML,
    UNKNOWN
};

ArchiveType ArchiveTypeFromExt(const std::string &ext);

} // namespace detail

template<class T>
void Save(const boost::filesystem::path &p, const T &obj)
{
    using namespace std;
    using namespace boost::archive;
    using detail::ArchiveType;

    ArchiveType archive_type = detail::ArchiveTypeFromExt(p.extension().string());

    if(archive_type == ArchiveType::UNKNOWN) {

        ELM_THROW_VALUE_ERROR("Unsupported extension.");
    }

    ofstream stream(p.string().c_str());

    if(!stream.is_open()) {

        std::stringstream s;
        s << "Failed to open file (" << p.string() << ") for writing.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    switch(archive_type) {

    case ArchiveType::BIN:
        {
        binary_oarchive oa(stream);
        oa << obj;
        }
        break;
    case ArchiveType::TXT:
        {
        text_oarchive oa(stream);
        oa << obj;
        }
        break;
    case ArchiveType::XML:
        {
        xml_oarchive oa(stream);
        oa << BOOST_SERIALIZATION_NVP(obj);
        }
        break;
    default:
        ELM_THROW_VALUE_ERROR("Unsupported extension.");
    }
}

template<class T>
void Load(const boost::filesystem::path &p, T &obj)
{
    using namespace std;
    using namespace boost::archive;
    using detail::ArchiveType;

    ArchiveType archive_type = detail::ArchiveTypeFromExt(p.extension().string());

    if(archive_type == ArchiveType::UNKNOWN) {

        ELM_THROW_VALUE_ERROR("Unsupported extension.");
    }

    ifstream stream(p.string().c_str());

    if(!stream.is_open()) {

        std::stringstream s;
        s << "Failed to open file (" << p.string() << ") for reading.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    switch(archive_type) {

    case ArchiveType::BIN:
        {
        binary_iarchive ia(stream);
        ia >> obj;
        }
        break;
    case ArchiveType::TXT:
        {
        text_iarchive ia(stream);
        ia >> obj;
        }
        break;
    case ArchiveType::XML:
        {
        xml_iarchive ia(stream);
        ia & BOOST_SERIALIZATION_NVP(obj);
        }
        break;
    default:
        ELM_THROW_VALUE_ERROR("Unsupported extension.");
    }
}

} // namespace elm

#endif // _ELM_IO_FILE_SERIALIZATION_H_
