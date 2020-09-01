#ifndef CARTESIO_VISUALSERVOING_H
#define CARTESIO_VISUALSERVOING_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>



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

        };



        class VisualServoingImpl: public virtual VisualServoingTask,
                                  public TaskDescriptionImpl
        {
            public:
                CARTESIO_DECLARE_SMART_PTR(VisualServoingImpl)

                VisualServoingImpl(YAML::Node node, Context::ConstPtr context);

                const std::string& getBaseLink() const;
                const std::string& getDistalLink() const;
                const std::string& getFeatureType() const;

            private:
                std::string _base_link;
                std::string _distal_link;
                std::string _feature_type;
        };



        class VisualServoingRos : public ServerApi::TaskRos
        {

        public:


            CARTESIO_DECLARE_SMART_PTR(VisualServoingRos)

            VisualServoingRos(TaskDescription::Ptr task, RosContext::Ptr context);

            void run(ros::Time time) override;

        private:

            VisualServoingTask::Ptr _ci_vs;
        };



        class VisualServoingRosClient : public ClientApi::TaskRos,
                virtual public VisualServoingTask
        {

        public:


            CARTESIO_DECLARE_SMART_PTR(VisualServoingRosClient)

            VisualServoingRosClient(std::string name, ros::NodeHandle nh);


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
