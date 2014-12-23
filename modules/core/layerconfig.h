#ifndef SEM_CORE_LAYERCONFIG_H_
#define SEM_CORE_LAYERCONFIG_H_

#include <string>
#include <vector>
#include <map>

#include <boost/property_tree/ptree.hpp>

typedef boost::property_tree::ptree PTree;
typedef std::vector<std::string> VecS;
typedef std::map<std::string, std::string> MapSS;

/**
 * @brief class for encapsulating layer IO keys
 */
class LayerIO
{
public:
    /**
     * @brief Set input pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Input(const std::string &key, const std::string &name);

    /**
     * @brief Set output pair. Overwrite if key exists.
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
     * @brief Get name to output feature  key
     * @param output feature key
     * @return
     * @throw Key Error if key does not exist.
     */
    std::string Output(const std::string &key) const;

private:
    MapSS inputs_;
    MapSS outputs_;
};

class LayerConfig : public LayerIO
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

#endif // SEM_CORE_LAYERCONFIG_H_
