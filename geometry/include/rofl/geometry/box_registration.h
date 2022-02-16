/**
 * ROFL - RIMLab Open Factotum Library
 * Copyright (C) 2021 Dario Lodi Rizzini, Ernesto Fontana
 *
 * ROFL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROFL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROFL.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef ROFL_GEOMETRY_BOX_REGISTRATION_H_
#define ROFL_GEOMETRY_BOX_REGISTRATION_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rofl/geometry/types.h>
#include <rofl/common/macros.h>

namespace rofl {

    /**
     * Class BoxRegistration enables registration of a box with DIM dimensions.
     * A box is modeled by edges parallel to the axes and corresponding to intervals
     * 
     *   [-0.5 * dimensions_(0), +0.5 * dimensions_(0)] x ... x  [-0.5 * dimensions_(DIM-1), +0.5 * dimensions_(DIM-1)]
     * 
     * The box is defined to be matched and aligned ("registered") to the point cloud. 
     * The registration is performed according to Iterative Closest Point (ICP). 
     */
    template <typename Scalar, int DIM>
    class BoxRegistration {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Definitions (the old "typedef")
        using Point = Eigen::Matrix<Scalar, DIM, 1>;
        using VectorPoint = std::vector<Point, Eigen::aligned_allocator<Point> >;
        using Transform = Eigen::Transform<Scalar, DIM, Eigen::Affine>;
        using SquareMatrix = Eigen::Matrix<Scalar, DIM, DIM>;
        using DynamicMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
        using Indices = Eigen::Matrix<int, DIM, 1>;
        using PermutationMatrix = Eigen::PermutationMatrix<DIM, DIM>;

        /**
         * Default constructor
         */
        BoxRegistration() : dimensions_(Point::Zero()), perm_() {
        }

        /**
         * Destructor. 
         */
        virtual ~BoxRegistration() {
        }

        /**
         * Sets the dimensions of the box. 
         * @param dim dimension vector with one dimension/size per axis
         */
        void setDimension(const Point& dim) {
            dimensions_ = dim;
            perm_.setIdentity();
            std::sort(perm_.indices().data(), perm_.indices().data() + DIM, [&](int i1, int i2) {
                return (dim(i1) < dim(i2));
            });
            perm_ = perm_.inverse();
            ROFL_MSG("dim = [" << dim.transpose() << "]\n"
                    "perm_.indices() [" << perm_.indices().transpose() << "]\n"
                    "permuted vector: perm_ * dim = [" << (perm_ * dim).transpose() << "]");
        }

        /**
         * Returns the point of the box that is nearest to the given query point. 
         * @param query the query point
         * @param nearest the found point of the box nearest to the query point
         */
        virtual void associate(const Point& query, Point& nearest) const;

        /**
         * Computes an updated estimation of the isometric transformation (rotation 
         * and translation) that aligns the box to the input point cloud. 
         * @param cloud the point cloud which is the target of registration
         * @param transf the transformation that aligns the box to the input point cloud
         */
        virtual void computeTransform(const VectorPoint& cloud, Transform& transf);

        /**
         * Estimates the initial transformation using centroid and moments. 
         * @param cloud the input point cloud
         * @param transf the initial estimation of transformation 
         */
        virtual void estimateInitTranform(const VectorPoint& cloud, Transform& transf) const;

    private:
        Point dimensions_;
        PermutationMatrix perm_;
    };

    template <typename Scalar, int DIM>
    void BoxRegistration<Scalar, DIM>::associate(const Point& query, Point& nearest) const {
        Scalar dist, edge;
        Scalar distNearest, edgeNearest;
        int dimNearest;
        bool inside;

        // Visits all the dimensions of the box
        inside = true;
        dimNearest = -1;
        for (int d = 0; d < DIM; ++d) {
            // Three cases: for each dimension 
            // a) coordinate query
            if (query(d) < -0.5 * dimensions_(d)) {
                nearest(d) = -0.5 * dimensions_(d);
                inside = false;
            } else if (query(d) > +0.5 * dimensions_(d)) {
                nearest(d) = +0.5 * dimensions_(d);
                inside = false;
            } else {
                nearest(d) = query(d);
                if (query(d) < 0.0) {
                    edge = -0.5 * dimensions_(d);
                    dist = query(d) - edge;
                } else {
                    edge = +0.5 * dimensions_(d);
                    dist = edge - query(d);
                }

                if (dimNearest < 0 || dist < distNearest) {
                    edgeNearest = edge;
                    distNearest = dist;
                    dimNearest = d;
                }
            }
        }

        if (inside) {
            ROFL_ASSERT_VAR1(dimNearest >= 0, dimNearest);
            nearest = query;
            nearest(dimNearest) = edgeNearest;
        }
    }

    template <typename Scalar, int DIM>
    void BoxRegistration<Scalar, DIM>::computeTransform(const VectorPoint& cloud, Transform& transf) {
        VectorPoint pointsModel;
        Transform transfInv = transf.inverse();
        Point pointNearest, p;
        Point meanSrc, meanDst, transl;
        //Eigen::MatrixXf S = Eigen::MatrixXf::Zero(DIM, DIM);
        DynamicMatrix S = DynamicMatrix::Zero(DIM, DIM);
        SquareMatrix R;

        // Associates each point of the cloud to its nearest neighbor on the box
        int n = cloud.size();
        pointsModel.reserve(n);
        for (int i = 0; i < n; ++i) {
            associate(transfInv * cloud[i], pointNearest);
            pointsModel.push_back(pointNearest);
        }

        // Estimates the new transformation solving the Procrustes problem 
        meanSrc = Point::Zero();
        meanDst = Point::Zero();
        for (int i = 0; i < n; ++i) {
            meanSrc += pointsModel[i];
            meanDst += cloud[i];
        }
        meanSrc = meanSrc * 1.0 / n;
        meanDst = meanDst * 1.0 / n;
        //        std::cout << "meanSrc " << meanSrc.transpose() << ", meanDst " << meanDst.transpose() << std::endl;

        for (int i = 0; i < n; ++i) {
            //std::cout << "matrix S: " << i << " cloud " << cloud[i].transpose() << " to model " <<  pointsModel[i].transpose() << "\n";
            S += (pointsModel[i] - meanSrc) * (cloud[i] - meanDst).transpose();
        }
        Eigen::JacobiSVD<DynamicMatrix> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // The matrix V * U' is guaranteed to be an isometry, but does not 
        float d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
        if (d > 0)
            d = 1.0;
        else
            d = -1.0;
        SquareMatrix I = SquareMatrix::Identity();
        I(1, 1) = d;

        R = svd.matrixV() * I * svd.matrixU().transpose();
        transl = meanDst - R * meanSrc;
        //        std::cout << "R\n" << R << std::endl;
        //        std::cout << "transl " << std::endl << transl << std::endl;

        // Puts the values into Transform2 data type
        transf.linear() = R;
        transf.translation() = transl;
        transf.makeAffine();
    }

    template <typename Scalar, int DIM>
    void BoxRegistration<Scalar, DIM>::estimateInitTranform(const VectorPoint& cloud, Transform& transf) const {
        Point centroid;
        SquareMatrix covar;

        // Estimates the centroid to set the translation
        centroid = Point::Zero();
        int n = cloud.size();
        if (n < 2) {
            transf = Transform::Identity();
            return;
        }
        for (int i = 0; i < n; ++i) {
            centroid += cloud[i];
        }
        centroid = centroid * 1.0 / n;

        // Estimates the covariance matrix: the initial rotation matrix corresponds 
        // to eigenvector matrix
        covar = SquareMatrix::Identity();
        for (int i = 0; i < n; ++i) {
            covar += (cloud[i] - centroid) * (cloud[i] - centroid).transpose();
        }
        covar = covar * 1.0 / (n - 1);

        Eigen::SelfAdjointEigenSolver<SquareMatrix> eigenSolver(covar);

        std::cout << "eigen values: " << eigenSolver.eigenvalues().transpose() << "\n"
                << "eigen vectors:\n" << eigenSolver.eigenvectors() << "\n";

        transf.linear() = eigenSolver.eigenvectors() * perm_;
        transf.translation() = centroid;
        transf.makeAffine();
    }


    using BoxRegistration2f = BoxRegistration<float, 2>;
    using BoxRegistration2d = BoxRegistration<double, 2>;
    using BoxRegistration3f = BoxRegistration<float, 3>;
    using BoxRegistration3d = BoxRegistration<double, 3>;

} // end of namespace

#endif /* BOXREGISTRATION_H */

