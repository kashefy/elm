/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include <fstream>

#include <boost/algorithm/string.hpp>   // tolower()

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/filesystem.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "elm/core/exception.h"
#include "elm/core/boost/serialization/serialization_utils.h"
#include "elm/layers/layerfactory.h"
#include "elm/layers/layergraph_impl.h"

namespace bfs=boost::filesystem;
using namespace std;
using namespace elm;

LayerGraph::~LayerGraph()
{
    if(impl_ != NULL) {

        delete impl_;
    }
    impl_ = NULL;
}

LayerGraph::LayerGraph()
    : impl_(new LayerGraph_Impl)
{
}

void LayerGraph::Add(const string &name,
                     LayerShared &layer,
                     const LayerConfig &cfg,
                     const LayerIONames &io) {

    impl_->Add(name, layer, cfg, io);
}

void LayerGraph::Add(const string &name,
                     const LayerConfig &cfg,
                     const LayerIONames &io) {

    impl_->Add(name, LayerFactory::CreateShared(name), cfg, io);
}

void LayerGraph::AddOutput(const string &output_name) {

    impl_->AddOutput(output_name);
}

SetS LayerGraph::Outputs() const {

    return impl_->Outputs();
}

SetS LayerGraph::Inputs() const {

    return impl_->Inputs();
}

void LayerGraph::ClearActive() {

    impl_->ClearActive();
}

void LayerGraph::Configure() {

    impl_->Configure();
}

void LayerGraph::Sequence(vector<LayerShared> &layer_seq) {

    impl_->Sequence(layer_seq);
}

size_t LayerGraph::num_layers() const {

    return impl_->num_layers();
}

void LayerGraph::ActivateForResponse(const vector<LayerShared> &layers, Signal &signal) {

    for(auto const& l : layers) {

        l->Activate(signal);
        l->Response(signal);
    }
}

void LayerGraph::Save(const string &file_path) const {

    bfs::path p(file_path);

    if(bfs::is_directory(p)) {

        stringstream s;
        s << "Path to regular file required for saving layer graph. " <<
             "Encountered path to directory " << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    ofstream stream(p.string().c_str());
    boost::archive::xml_oarchive oa(stream);

    oa & boost::serialization::make_nvp("graph", impl_->g_);
}

void LayerGraph::SaveJSON(const string &file_path) const {

    bfs::path p(file_path);

    if(bfs::is_directory(p)) {

        stringstream s;
        s << "Path to regular file required for saving layer graph. " <<
             "Encountered path to directory " << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    string ext = p.extension().string();
    boost::algorithm::to_lower(ext);

    if(ext != ".json") {

        stringstream s;
        s << p.string() << " is not a .json file.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    PTree tree;
    impl_->to_ptree(tree);

    boost::property_tree::json_parser::write_json(p.string(), tree);
}

void LayerGraph::Load(const string &file_path) {

    bfs::path p(file_path);

    if(bfs::is_directory(p)) {

        stringstream s;
        s << "Path to regular file required to load layer graph. " <<
             "Encountered path to directory " << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }
    else if(!bfs::is_regular_file(p)) {

        stringstream s;
        s << "Cannot load layer graph from" << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    ifstream stream(p.string().c_str());
    boost::archive::xml_iarchive oa(stream);

    oa & boost::serialization::make_nvp("graph", impl_->g_);
}

void LayerGraph::LoadJSON(const string &file_path) {

    bfs::path p(file_path);

    if(bfs::is_directory(p)) {

        stringstream s;
        s << "Path to regular file required to load layer graph. " <<
             "Encountered path to directory " << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }
    else if(!bfs::is_regular_file(p)) {

        stringstream s;
        s << "Cannot load layer graph from" << p;
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    string ext = p.extension().string();
    boost::algorithm::to_lower(ext);

    if(ext != ".json") {

        stringstream s;
        s << p.string() << " is not a .json file.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    PTree tree;
    boost::property_tree::json_parser::read_json(p.string(), tree);

    PTree tree_vtx = tree.get_child("layers");
    for(PTree::const_iterator itr=tree_vtx.begin(); itr != tree_vtx.end(); ++itr) {

        string tmp = itr->first;

        if(!tmp.empty()) {

            stringstream s;
            s << "Unexpected key \'" << tmp << "\'" <<
                 " in list of layers in json file (" << file_path << ").";
            ELM_THROW_KEY_ERROR(s.str());
        }

        PTree ptree_vtx = itr->second;
        //PrintXML(ptree_vtx, std::cout); std::cout<<std::endl;
        LayerVertex vtx_info;
        vtx_info.from_ptree(ptree_vtx);

        Add(vtx_info.name, vtx_info.cfg, vtx_info.io);
    }
}

// template specializations for LayerGraph::Reconfigure()
template <>
int LayerGraph::Reconfigure<bool>(const string &key, const bool& value) {

    return impl_->Reconfigure<bool>(key, value);
}

template <>
int LayerGraph::Reconfigure<float>(const string &key, const float& value) {

    return impl_->Reconfigure<float>(key, value);
}

template <>
int LayerGraph::Reconfigure<int>(const string &key, const int& value) {

    return impl_->Reconfigure<int>(key, value);
}

template <>
int LayerGraph::Reconfigure<string>(const string &key, const string& value) {

    return impl_->Reconfigure<string>(key, value);
}
