/** Classes for reading MNIST database files
 *  source: http://yann.lecun.com/exdb/mnist/
 */
#ifndef SEM_IO_READMNIST_H_
#define SEM_IO_READMNIST_H_

#include <string>
#include <fstream>

/**
 * @brief class for reading MNIST label data
 */
class ReadMNISTLabel
{
public:
    ReadMNISTLabel(const std::string &path);

    unsigned char Next();

protected:
    std::ifstream input_; ///< input streams
    int nb_items_;        ///< total no. of items

    const int MAGIC_NUMBER_ = 2049; ///< used to validate file integrity, see MNIST site
};

#endif // SEM_IO_READMNIST_H_
