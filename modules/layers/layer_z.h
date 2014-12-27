#ifndef SEM_LAYERS_LAYER_Z_H_
#define SEM_LAYERS_LAYER_Z_H_

#include <vector>

#include "core/base_Layer.h"
#include "neuron/zneuron.h"
#include "neuron/wtapoisson.h"

/**
 * @brief Class for defining learning nodes as in the SEM NIPS paper 2010
 *
 *  A set of spiking neurons in a WTA circuit that learn via STDP
 *
 * @todo proper citation
 */
class LayerZ : public base_LearningLayer
{
public:
    // I/O keys
    static const std::string KEY_INPUT_SPIKES;        ///< key to input spikes
    static const std::string KEY_OUTPUT_SPIKES;       ///< key to output spikes
    static const std::string KEY_OUTPUT_MEMBRANE_POT; ///< key to neuron membrane potentials
    static const std::string KEY_OUTPUT_WEIGHTS;      ///< key to neuron weights
    static const std::string KEY_OUTPUT_BIAS;         ///< key to neuron bias

    // Parmater keys, parameters with defaults are optional
    static const std::string PARAM_NB_AFFERENTS;      ///< no. of afferent inputs
    static const std::string PARAM_NB_OUTPUT_NODES;   ///< gabor envelope sigma
    static const std::string PARAM_LEN_HISTORY;       ///< length of spiking histry to maintain
    static const std::string PARAM_DELTA_T;           ///< spike time resolution [milliseconds]
    static const std::string PARAM_WTA_FREQ;          ///< WTA's  spiking frequency [Hz]

    // defaults, parameters with defaults are optional
    static const int DEFAULT_LEN_HISTORY;             ///< 5, not a time unit, @todo change to time unit
    static const float DEFAULT_DELTA_T;               ///< = 1000.f;
    static const float DEFAULT_WTA_FREQ;              ///< = 1.f; // 1 Hz

    ~LayerZ();

    LayerZ();

    /**
     * @brief Configurable constructor, set parameters and IO
     * @param config
     */
    LayerZ(const LayerConfig& config);

    /**
     * @brief Clear layer state
     */
    void Clear();

    /**
     * @brief Reset and override layer configuration
     * @param config
     */
    void Reset(const LayerConfig &config);

    /**
     * @brief Reconfigure, soft reset
     * @param config
     */
    void Reconfigure(const LayerConfig &config);

    /**
      * @brief Set layer input output keys
      * @param new I/O configuration
      */
    virtual void IONames(const LayerIONames& config);

    /**
     * @brief Set stimulus
     * @param signal containing stimuli
     */
    void Stimulus(const Signal &signal);

    /**
     * @brief Apply layer to most recent stimuli
     */
    void Apply();

    /**
     * @brief Learn from most recent stimuli
     */
    void Learn();

    /**
     * @brief Get response due to most recent stimuli application
     * @param signal for encapsulating response
     */
    void Response(Signal &signal);

protected:
    typedef std::vector<std::shared_ptr<base_Learner> > VecLPtr; ///< vector typedef convinience

    /**
     * @brief initialize vector of learners
     * @param no. of features
     * @param history length
     */
    void InitLearners(int nb_features, int nb_outputs, int len_history);

    std::string name_input_spikes_;     ///< name of input spikes in signal object
    std::string name_output_spikes_;    ///< destination of output spikes in signal object
    OptS name_output_mem_pot_;          ///< optional destination of membrane potential in signal object
    OptS name_output_weights_;          ///< optional destination of neuron weights in signal object
    OptS name_output_bias_;             ///< optional destination of neuron bias in signal object, not the same as weights[0]

    int nb_afferents_;                  ///< number of afferents to this layer

    cv::Mat1f spikes_in_;               ///< stimulus
    cv::Mat1f u_;                       ///< membrane potential from most recent stimuli
    cv::Mat1f spikes_out_;              ///< output spikes from most recent stimuli

    VecLPtr z_;                         ///< z neurons that learn using STDP
    WTAPoisson wta_;                    ///< winner-take-all to govern Z neuron spiking
};

#endif // SEM_LAYERS_LAYER_Z_H_
