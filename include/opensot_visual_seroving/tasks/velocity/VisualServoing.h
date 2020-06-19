#ifndef __TASKS_VELOCITY_VISUAL_SERVOING_H__
#define __TASKS_VELOCITY_VISUAL_SERVOING_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/utils/Piler.h>
#include <visp/vpBasicFeature.h>

using namespace XBot::Utils;

namespace OpenSoT {
   namespace tasks {
       namespace velocity {
       class VisualServoing: public Cartesian {
         public:
            typedef boost::shared_ptr<VisualServoing> Ptr;

           /**
             * @brief VisualServoing constructor
             * @param task_id name of the visual servoing task
             * @param x robot configuration
             * @param robot model
             * @param base_link of the visual servoing task
             * @param camera_link controlled camera link
             */
            VisualServoing(std::string task_id,
                           const Eigen::VectorXd& x,
                           XBot::ModelInterface &robot,
                           std::string base_link,
                           std::string camera_link);


            /**
             * @brief setVelocityTwistMatrix sets adjoint matrix between sensor_frame and camera_frame
             * @param V adjoint matrix
             */
            void setVelocityTwistMatrix(const Eigen::Matrix6d& V);
            void getVelocityTwistMatrix(Eigen::Matrix6d& V);

            void addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select = vpBasicFeature::FEATURE_ALL)

            {
              _featureList.push_back(&s_cur);
              _desiredFeatureList.push_back(&s_star);
              _featureSelectionList.push_back(select);
            }

            //bool updateFeatures();

            static void computeInteractionMatrixFromList(const std::list<vpBasicFeature *> &featureList,
                                                         const std::list<unsigned int> &featureSelectionList, vpMatrix &L);

       private:
            void _update(const Eigen::VectorXd& x);

            /**
             * @brief _J stores temporary Jacobian from Cartesian task
             */
            Eigen::MatrixXd _J;

            /**
             * @brief _v stores temporary velocity references from Cartesian task
             */
            Eigen::VectorXd _v;

            /**
             * @brief _V constant adjoint matrix which stores the offset between sensor_frame and camera_frame
             */
            Eigen::Matrix6d _V;

            std::list<vpBasicFeature *> _featureList;
            std::list<vpBasicFeature *> _desiredFeatureList;
            std::list<unsigned int> _featureSelectionList;

            /**
             * @brief _L interaction matrix
             */
            Eigen::MatrixXd _L;
            vpMatrix _L_visp;

            void computeInteractionMatrix();

            template<typename Derived>
            void visp2eigen(const vpMatrix &src, Eigen::MatrixBase<Derived> &dst)
            {
              dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(src.data, src.getRows(), src.getCols());
            }


       };
       }
   }
}

#endif
