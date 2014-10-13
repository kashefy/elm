/** Classes for reading MNIST database files
 *  source: http://yann.lecun.com/exdb/mnist/
 */
#ifndef SEM_IO_READMNIST_H_
#define SEM_IO_READMNIST_H_

#include <string>
#include <fstream>

#include <opencv2/core.hpp>

/**
 * @brief base class for reading MNIST files
 */
class base_ReadMNISTFile
{
public:
    virtual ~base_ReadMNISTFile();

    /**
     * @brief Get magic number expected in file header
     * with which we verify the file's integrity
     * @return expected number in file header
     */
    virtual int MagicNumber() const = 0;

    /**
     * @brief read file header
     * @param path
     * @return no. of items
     */
    virtual int ReadHeader(const std::string &path);

    /**
     * @brief read next item
     * @return item value encapsulated in a matrix object
     */
    virtual cv::Mat Next() = 0;

    /**
     * @brief Check if we've reached the end of the file
     * @return true on end of file reached
     */
    virtual bool IS_EOF() const;

protected:
    base_ReadMNISTFile();

protected:
    std::ifstream input_; ///< input stream
    int nb_items_;        ///< total no. of items
};

/**
 * @brief class for reading MNIST label data
 */
class ReadMNISTLabels : public base_ReadMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2049; ///< used to validate file integrity, see MNIST site

    ReadMNISTLabels();

    virtual int MagicNumber() const;

    virtual cv::Mat Next();
};

class ReadMNISTImages : public base_ReadMNISTFile
{
public:
    static const int MAGIC_NUMBER = 2051; ///< used to validate file integrity, see MNIST site

    ReadMNISTImages();

    virtual int MagicNumber() const;

    virtual cv::Mat Next();
};

#endif // SEM_IO_READMNIST_H_
