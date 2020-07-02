#ifndef __TASKS_VELOCITY_VISUAL_SERVOING_H__
#define __TASKS_VELOCITY_VISUAL_SERVOING_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/utils/Piler.h>
#include <visp/vpBasicFeature.h>

using namespace XBot::Utils;

namespace OpenSoT {
   namespace tasks {
       namespace velocity {
       class VisualServoing: public Task < Eigen::MatrixXd, Eigen::VectorXd > {
         public:
            typedef boost::shared_ptr<VisualServoing> Ptr;

           /**
             * @brief VisualServoing constructor
             * @param task_id name of the visual servoing task
             * @param x robot configuration
             * @param robot model
             * @param base_link of the visual servoing task
             * @param camera_link controlled camera link
             * @param features to track
             */
            VisualServoing(std::string task_id,
                           const Eigen::VectorXd& x,
                           XBot::ModelInterface &robot,
                           std::string base_link,
                           std::string camera_link,
                           std::list<vpBasicFeature *>& feature_list);


            /**
             * @brief setVelocityTwistMatrix used to set adjoint matrix between sensor_frame and camera_frame
             * @param V [6x6] adjoint matrix
             */
            void setVelocityTwistMatrix(const Eigen::Matrix6d& V);

            /**
             * @brief getVelocityTwistMatrix used to get adjoint matrix between sensor_frame and camera_frame
             * @return [6x6] adjoint matrix
             */
            const Eigen::Matrix6d& getVelocityTwistMatrix() const;

            /**
             * @brief getInteractionMatrix used to get computed interaction matrix
             * @return [kx6] interaction matrix, with k proportional to number of features
             */
            const Eigen::MatrixXd& getInteractionMatrix() const;

            /**
             * @brief addFeature used to insert a new couple of feature and desired feature
             * @param s_cur feature
             * @param s_star desired feature
             * @param select TODO: test this param!
             * NOTE: every time a new feature is added the intearaction matrix is computed and Weight matrix is
             * set to Identity
             */
            void addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select = vpBasicFeature::FEATURE_ALL);

            /**
             * @brief setFeatures used to set a list of new features and desired features
             * @param feature_list
             * @param desired_feature_list
             * @param feature_selection_list
             * @return false if inputs have different size
             * NOTE: every time a new feature is added the intearaction matrix is computed and Weight matrix is
             * set to Identity
             */
            bool setFeatures(std::list<vpBasicFeature *>& feature_list,
                             std::list<vpBasicFeature *>& desired_feature_list,
                             std::list<unsigned int>& feature_selection_list);

            /**
             * @brief setDesiredFeatures is used to change the desired feature list
             * @param desired_feature_list list of desired features
             * @return false if the lenght of desired features is different from the actual lenght of features
             */
            bool setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list);

            /**
             * @brief getFeatures used to get feature list
             * @return list of features
             */
            const std::list<vpBasicFeature *>& getFeatures() const;

            /**
             * @brief getDesiredFeatures used to get desired feature list
             * @return list of desired features
             */
            const std::list<vpBasicFeature *>& getDesiredFeatures() const;


            /**
             * @brief computeInteractionMatrixFromList used to compyte interaction matrix
             * @param featureList list of features
             * @param featureSelectionList list of feature selectors
             * @param L interaction matrix
             */
            static void computeInteractionMatrixFromList(const std::list<vpBasicFeature *> &featureList,
                                                         const std::list<unsigned int> &featureSelectionList, vpMatrix &L);

            /**
             * @brief getBaseLink returns base link wrt jacobian is computed
             * @return base link
             */
            const std::string& getBaseLink() const;

            /**
             * @brief getCameraLink return controlled camera link
             * @return camera link
             */
            const std::string& getCameraLink() const;

            /**
             * @brief setEyeInHand: controll is computed for a camera mounted on the moving link
             */
            void setEyeInHand();

            /**
             * @brief isEyeInHand
             * @return true or false
             */
            bool isEyeInHand();

            /**
             * @brief setEyeToHand: control is computed for a camera looking at the moving link
             */
            void setEyeToHand();

            /**
             * @brief isEyeToHand
             * @return true or false
             */
            bool isEyeToHand();

            friend VisualServoing::Ptr operator%(const VisualServoing::Ptr task, const std::list<unsigned int>& rowIndices);

       private:
            void _update(const Eigen::VectorXd& x);

            void compute_b();

            /**
             * @brief _J stores temporary Jacobian from Cartesian task
             */
            Eigen::MatrixXd _J;


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

            template<typename Derived>
            void visp2eigen(const vpColVector& src, Eigen::MatrixBase<Derived> &dst)
            {
                dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(src.data, src.getRows());
            }

            /**
             * @brief _cartesian_task internal, use for computation purposes
             */
            Cartesian::Ptr _cartesian_task;

            /**
             * @brief _eye_in_hand, if false means: eye_to_hand
             */
            bool _eye_in_hand;

            std::list<unsigned int> _row_indices;


       };

       /**
        * @brief operator % Needs to be redefined for the VisualServoing task in order to have that the % operation
        * remove Cartesian direction instead of features rows.
        * At the moment the method is implemented so that columns of the interaction matrix are set to zero.
        * TODO:
        *   test the use of _featureSelectionList
        *
        * @param task
        * @param rowIndices
        * @return task
        */
       VisualServoing::Ptr operator%(const VisualServoing::Ptr task, const std::list<unsigned int>& rowIndices);
       }
   }
}

#endif
