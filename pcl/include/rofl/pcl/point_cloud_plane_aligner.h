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
#ifndef ROFL_PCL_POINT_CLOUD_PLANE_ALIGNER_H_
#define ROFL_PCL_POINT_CLOUD_PLANE_ALIGNER_H_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>


namespace rofl {

    /**
     * Class PointCloudPlaneAligner computes a reference frame R consistent with
     * the input plane P, i.e. the normal of P is aligned with the axis Z of R. 
     * It also transforms the input point clouds to this reference frame. 
     */
    template <typename PointT>
    class PointCloudPlaneAligner {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;
        using PointCloudPtr = typename PointCloud::Ptr;

        /**
         * Constructor. 
         */
        PointCloudPlaneAligner() {
        }

        /**
         * Destructor. 
         */
        virtual ~PointCloudPlaneAligner() {
        }

        /**
         * Sets the input cloud.
         */
        void setInputCloud(PointCloudConstPtr cloudIn) {
            cloud_ = cloudIn;
        }

        /**
         * Sets the input plane coefficients with the following meaining.
         * Let p = [p_x, p_y, p_z, 1]^T be a point, the plane equation is 
         * 
         *   coeff^T * p = 0
         * 
         * The choice of the plane coefficients is not not unique. 
         * The function internally adjust the plane s.t. coeff(3) > 0, i.e. 
         * the normal of the plane "looks" toward the origin. 
         * If the point cloud is the sensor frame/original viewpoint, then the 
         * axis Z of the resulting frame looks toward the sensor. 
         */
        void setPlaneCoeffs(const Eigen::Vector4f& coeff) {
            if (coeff(3) > 0.0) {
                plane_ = coeff;
            } else {
                plane_ = -coeff;
            }
        }

        /**
         * Returns the plane coefficients. 
         */
        const Eigen::Vector4f& getPlaneCoeffs() const {
            return plane_;
        }

        /**
         * Returns the transformation.
         */
        const Eigen::Affine3f& getTransform() const {
            return transform_;
        }

        /**
         * Computes the transformation for setting plane to 
         */
        void compute() {
            Eigen::Vector3f ax, ay, az, centerPlane;
            Eigen::Matrix3f Rot;
            Eigen::Vector4f centroid;
            float planeConst;
            int idxMax;

            // Sets the plane normal (plane coeffs 0:2) as axis Z
            az = plane_.head<3>();
            az.normalize();
            az.cwiseAbs().maxCoeff(&idxMax);
            planeConst = -plane_(3);

            // Choose axis X according to the maximum component of plane normal
            switch (idxMax) {
                case 0: ax << 0.0, 0.0, 1.0;
                    break;
                case 1: ax << 0.0, 0.0, 1.0;
                    break;
                case 2:
                default: ax << 1.0, 0.0, 0.0;
                    break;
            }
            // Orthonormalization of axis X and computation of axis Y 
            ax = ax - ax.dot(az) * az;
            ax.normalize();
            ay = az.cross(ax);
            Rot << ax, ay, az;
            //std::cout << "Rot\n" << Rot << "\n ax.cross(ay) " << ax.cross(ay).transpose() << std::endl;

            pcl::compute3DCentroid(*cloud_, centroid);
            centerPlane = centroid.head<3>() - centroid.dot(plane_) * az;
            //std::cout << "centerPlane " << centerPlane.transpose() << ": n^T * centerPlane - d = " << (az.dot(centerPlane) - planeConst) << std::endl;

            transform_ = Eigen::Affine3f::Identity();
            transform_.pretranslate(-centerPlane);
            transform_.prerotate(Rot.inverse());
            //transform_.linear().topLeftCorner<3,3>() = Rot;
            //transform_.translation() = -centerPlane;

            //std::cout << "Transform:\n" << transform_.matrix() << std::endl;
        }

        void transform(PointCloud& cloudOut) const {
            pcl::transformPointCloud(*cloud_, cloudOut, transform_);
        }

    private:
        PointCloudConstPtr cloud_;
        Eigen::Vector4f plane_;
        Eigen::Affine3f transform_;
    };

}  // end of namespace 

#define PCL_INSTANTIATE_PointCloudPlaneAligner(T) template class PCL_EXPORTS rofl::PointCloudPlaneAligner<T>;


#endif /* POINTCLOUDPLANEALIGNER_H */

