#ifndef SEM_CORE_SIGNAL_H_
#define SEM_CORE_SIGNAL_H_

#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

/**
 * @brief The Signal class, a class for holding single and multiple samples of features
 */
class Signal
{
public:
    typedef std::vector<cv::Mat> VecMat;

    ~Signal();

    Signal();

    /**
     * @brief Append signal to an existing key. Will add if it doesn't exist.
     * @param name or key
     * @param feature data to append
     */
    void Append(const std::string &name, const cv::Mat &feature_data);

    /**
     * @brief Check if a signal exists under a given name
     * @param name/key
     */
    void Exists(std::string names) const;

protected:
    std::map<std::string, VecMat> signals_; ///< encapuslated signals
};

#endif // SEM_CORE_SIGNAL_H_
