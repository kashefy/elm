#ifndef _ELM_LAYERS_LAYERVERTEX_H_
#define _ELM_LAYERS_LAYERVERTEX_H_

#include <memory>

#include <boost/property_tree/ptree_serialization.hpp>
#include <boost/serialization/map.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/typedefs_fwd.h"

namespace elm {

/** @brief wrap layer information to store as vertex
 */
struct LayerVertex {

    friend class boost::serialization::access;

    LayerVertex();

    /**
     * @brief Set necessary info
     * @param _cfg configuration
     * @param _io I/O names
     * @param _ptr pointer to instance
     */
    void Set(const std::string &_name,
             const LayerConfig &_cfg,
             const LayerIONames &_io,
             const LayerShared &_ptr);

    /**
     * @brief Configure layer
     */
    void Configure();

    /**
     * @brief Serialize information to property tree
     * @param[out] p destination property tree
     */
    void to_ptree(PTree &p) const;

    /**
     * @brief Initialize information from property tree
     * @param[in] p information source
     */
    void from_ptree(const PTree &p);

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {

        ar & BOOST_SERIALIZATION_NVP(name);

        PTree params = cfg.Params();
        ar & BOOST_SERIALIZATION_NVP(params);

        ar & boost::serialization::make_nvp("inputs", io.InputMap());
        ar & boost::serialization::make_nvp("outputs", io.OutputMap());
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version) {

        ar & BOOST_SERIALIZATION_NVP(name);

        PTree params;
        ar & BOOST_SERIALIZATION_NVP(params);
        cfg.Params(params);

        MapSS m;
        ar & boost::serialization::make_nvp("inputs", m);

        for(MapSS::const_iterator itr=m.begin(); itr != m.end(); ++itr) {

            io.Input(itr->first, itr->second);
        }

        m.clear();
        ar & boost::serialization::make_nvp("outputs", m);

        for(MapSS::const_iterator itr=m.begin(); itr != m.end(); ++itr) {

            io.Output(itr->first, itr->second);
        }
    }

    std::string name;   ///< layer name (e.g. class name)
    LayerConfig cfg;    ///< configuration
    LayerIONames io;    ///< I/O
    LayerShared ptr;    ///< pointer to shared instance
    bool is_active;     ///< flag if active or not
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERVERTEX_H_
