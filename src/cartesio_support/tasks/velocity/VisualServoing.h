#ifndef CARTESIO_VISUALSERVOING_H
#define CARTESIO_VISUALSERVOING_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>

#include <opensot_visual_servoing/VisualFeatures.h>

#include <visp/vpBasicFeature.h>




using VSSoT = OpenSoT::tasks::velocity::VisualServoing;

namespace XBot {
    namespace Cartesian {
        namespace velocity {

        const std::map<std::string, int> FEATURE_TYPE = {
          { "vpFeaturePoint", 2 }
        };


        class VisualServoingTask: public virtual TaskDescription
        {
            public:
                CARTESIO_DECLARE_SMART_PTR(VisualServoingTask)

                virtual const std::string& getBaseLink() const = 0;
                virtual const std::string& getDistalLink() const = 0 ;
                virtual const std::string& getFeatureType() const = 0;
                virtual const std::list<vpBasicFeature * >& getFeatures() const = 0;
                virtual const std::list<vpBasicFeature * >& getDesiredFeatures() const = 0;
                virtual const std::list<unsigned int>& getFeatureSelectionList() const = 0;

                virtual void setVelocityTwistMatrix(const Eigen::Matrix6d& V) = 0;
                virtual void addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star,
                                        unsigned int select = vpBasicFeature::FEATURE_ALL) = 0;
                virtual bool setFeatures(std::list<vpBasicFeature *>& feature_list,
                                 std::list<vpBasicFeature *>& desired_feature_list,
                                 std::list<unsigned int>& feature_selection_list) = 0;
                virtual bool setFeatures(std::list<vpBasicFeature *>& feature_list) = 0;
                virtual bool setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list) = 0;
        };



        class VisualServoingImpl: public virtual VisualServoingTask,
                                  public TaskDescriptionImpl
        {
            public:
                CARTESIO_DECLARE_SMART_PTR(VisualServoingImpl)

                VisualServoingImpl(YAML::Node node, Context::ConstPtr context);

                const std::string& getBaseLink() const override;
                const std::string& getDistalLink() const override;
                const std::string& getFeatureType() const override;
                const std::list<vpBasicFeature * >& getFeatures() const override;
                const std::list<vpBasicFeature * >& getDesiredFeatures() const override;
                const std::list<unsigned int>& getFeatureSelectionList() const override;


                void setVelocityTwistMatrix(const Eigen::Matrix6d& V) override;
                void addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star,
                                unsigned int select = vpBasicFeature::FEATURE_ALL) override;
                bool setFeatures(std::list<vpBasicFeature *>& feature_list,
                         std::list<vpBasicFeature *>& desired_feature_list,
                         std::list<unsigned int>& feature_selection_list) override;
                bool setFeatures(std::list<vpBasicFeature *>& feature_list) override;
                bool setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list) override;

            private:
                std::string _base_link;
                std::string _distal_link;
                std::string _feature_type;

                Eigen::Matrix6d _V;
                std::list<vpBasicFeature *> _featureList, _desiredFeatureList;
                std::list<unsigned int> _featureSelectionList;
        };



        class VisualServoingRos : public ServerApi::TaskRos
        {

        public:


            CARTESIO_DECLARE_SMART_PTR(VisualServoingRos)

            VisualServoingRos(TaskDescription::Ptr task, RosContext::Ptr context);

            void run(ros::Time time) override;

        private:

            VisualServoingTask::Ptr _ci_vs;
            ros::Subscriber _feature_sub, _desired_feature_sub;

            std::list<vpBasicFeature*> getFeaturesFromMsg(opensot_visual_servoing::VisualFeaturesConstPtr msg);

            bool _visual_servoing_init;
        };




        class OpenSotVisualServoingAdapter : public OpenSotTaskAdapter
        {

        public:

            OpenSotVisualServoingAdapter(TaskDescription::Ptr ci_task, Context::ConstPtr context);

            virtual TaskPtr constructTask() override;

            virtual void update(double time, double period) override;

            virtual void processSolution(const Eigen::VectorXd& solution) override;

            virtual ~OpenSotVisualServoingAdapter() override = default;

        protected:

            VSSoT::Ptr _opensot_vs;

        private:
            VisualServoingTask::Ptr _ci_vs;

        };







        }
    }
}

#endif
