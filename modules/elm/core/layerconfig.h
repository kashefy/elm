#ifndef ELM_CORE_LAYERCONFIG_H_
#define ELM_CORE_LAYERCONFIG_H_

#include <string>
#include <vector>
#include <map>

#include <boost/property_tree/ptree.hpp>

typedef boost::property_tree::ptree PTree;
typedef boost::optional<std::string> OptS;
typedef std::vector<std::string> VecS;
typedef std::map<std::string, std::string> MapSS;

/**
 * @brief class for encapsulating layer IO key-name pairs
 */
class LayerIONames
{
public:
    /**
     * @brief Set input key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Input(const std::string &key, const std::string &name);

    /**
     * @brief Set output key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Output(const std::string &key, const std::string &name);

    /**
     * @brief Get name to input feature key
     * @param input feature key
     * @return name
     * @throw Key Error if key does not exist.
     */
    std::string Input(const std::string &key) const;

    /**
     * @brief Get name to optional input feature key
     * @param input feature key
     * @return name if exists, check existence using bool() and retrieve content via .get()
     */
    OptS InputOpt(const std::string &key) const;

    /**
     * @brief Get name to output feature key
     * @param output feature key
     * @return name
     * @throw Key Error if key does not exist.
     */
    std::string Output(const std::string &key) const;

    /**
     * @brief Get name to optional output feature key
     * @param output feature key
     * @return name if exists, check existence using bool() and retrieve content via .get()
     */
    OptS OutputOpt(const std::string &key) const;

private:
    MapSS inputs_;
    MapSS outputs_;
};

class LayerConfig : public LayerIONames
{
public:
    /**
     * @brief Default Constructor
     */
    LayerConfig();

    /**
     * @brief Set layer parameters
     * @param params
     */
    void Params(const PTree &params);

    /**
     * @brief Get parameters
     * @return layer parameters
     */
    PTree Params() const;

private:
    PTree params_;
};

#endif // ELM_CORE_LAYERCONFIG_H_
