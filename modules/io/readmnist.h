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
    static const int MAGIC_NUMBER = 2049; ///< used to validate file integrity, see MNIST site

    ReadMNISTLabel(const std::string &path);

    unsigned char Next();

    bool IS_EOF() const;

protected:
    std::ifstream input_; ///< input streams
    int nb_items_;        ///< total no. of items

};

#endif // SEM_IO_READMNIST_H_
