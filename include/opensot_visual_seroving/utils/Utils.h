#ifndef __UTILS_VISUAL_SERVOING_H__
#define __UTILS_VISUAL_SERVOING_H__

#include <Eigen/Core>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

template<typename Derived>
/**
 * @brief visp2eigen convert a visp matrix in eigen matrix
 * @param src vpMatrix
 * @param dst eigen matrix
 */
void visp2eigen(const vpMatrix &src, Eigen::MatrixBase<Derived> &dst)
{
    dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(src.data, src.getRows(), src.getCols());
}

template<typename Derived>
/**
 * @brief visp2eigen convert a visp vector in eigen vector
 * @param src vpColVector
 * @param dst eigen vector
 */
void visp2eigen(const vpColVector& src, Eigen::MatrixBase<Derived> &dst)
{
    dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(src.data, src.getRows());
}

template<typename Derived>
void eigen2visp(const Eigen::VectorXd &src, vpColVector &dst)
{
    dst.resize(static_cast<unsigned int>(src.rows()));
    #if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
    for (Eigen::DenseIndex i = 0; i < src.rows(); i++) {
    #else
    for (Eigen::Index i = 0; i < src.rows(); i++) {
    #endif
        dst[static_cast<unsigned int>(i)] = src(i);
    }
}

#endif
