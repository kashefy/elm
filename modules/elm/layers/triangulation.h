/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_TRIANGULATION_H_
#define _ELM_LAYERS_TRIANGULATION_H_

#ifndef __WITH_PCL
    #warning "Disabling Triangulation layer due to no PCL support and defining it as a non-supported"
    #include "elm/layers/layernotsupported.h"
    class Triangulation : public ELM_LAYER_NOT_SUPPORTED(Triangulation, "Building with PCL is required for supporting Triangulation layer.");
#else   // __WITH_PCL

#include <string>

#include <opencv2/core.hpp>

#include <pcl/surface/gp3.h>

#include "elm/core/base_Layer.h"

namespace elm {

/**
 * @brief The triangulation layer wraps around the greedy projection algorithm usign pcl @cite Marton09ICRA
 */
class Triangulation : public base_Layer
{
public:
    // Parameters
    static const std::string PARAM_SEARCH_RADIUS;       ///< maximum distance between connected points (maximum edge length)
    static const std::string PARAM_MU;
    static const std::string PARAM_MAX_NN;              ///< maximum neighrest neighbors
    static const std::string PARAM_MAX_SURFACE_ANGLE;   ///< maximum surface angle
    static const std::string PARAM_MIN_ANGLE;           ///< [rad]
    static const std::string PARAM_MAX_ANGLE;           ///< [rad]
    static const std::string PARAM_IS_NORMAL_CONSISTENT;///< yes/no

    // I/O names
    static const std::string KEY_INPUT_POINT_CLOUD;         ///< key to source cloud
    static const std::string KEY_OUTPUT_VERTICES;           ///< key to output vertices

    // Default parameter values
    static const float DEFAULT_SEARCH_RADIUS;               ///< 0.025
    static const float DEFAULT_MU;                          ///< 2.5
    static const float DEFAULT_MAX_SURFACE_ANGLE;           ///< PI/4 rad = 45 deg
    static const float DEFAULT_MIN_ANGLE;                   ///< PI/18 rad = 10 deg
    static const float DEFAULT_MAX_ANGLE;                   ///< 2/3*PI rad = 120 deg

    static const int  DEFAULT_MAX_NN;                       ///< 100
    static const bool DEFAULT_IS_NORMAL_CONSISTENCY;        ///< false

    Triangulation();

    Triangulation(const LayerConfig &cfg);

    void Clear();

    void Reset();

    void Reset(const LayerConfig &cfg);

    void Reconfigure(const LayerConfig &cfg);

    void InputNames(const LayerIONames &config);

    void OutputNames(const LayerIONames &config);

    void Activate(const Signal &signal);

    void Response(Signal &signal);

protected:
    std::string name_src_cloud_;   ///< name of input point cloud in signal object
    std::string name_vertices_;    ///< desitnation name of output vertices

    cv::Mat1f vertices_;            ///< populated with vertices of reconstructed triangular mesh
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   ///< greedy projection triangulation from point cloud
};

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_LAYERS_TRIANGULATION_H_
