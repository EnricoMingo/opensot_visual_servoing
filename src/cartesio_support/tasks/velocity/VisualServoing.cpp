#include "VisualServoing.h"
#include <visp/vpBasicFeature.h>

using namespace XBot::Cartesian::velocity;

namespace  {

std::string get_name(YAML::Node node)
{
    auto distal_link = node["distal_link"].as<std::string>();
    return "visual_servoing_" + distal_link;
}

int get_size(YAML::Node node)
{
    std::string feature_type = node["feature_type"].as<std::string>();
    int number_of_features = node["number_of_features"].as<int>();
    return FEATURE_TYPE.at(feature_type)*number_of_features;
}

}


VisualServoingImpl::VisualServoingImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, get_name(node), get_size(node))
{
    _base_link = node["base_link"].as<std::string>();
    _distal_link = node["distal_link"].as<std::string>();
    _feature_type = node["feature_type"].as<std::string>();
}

const std::string& VisualServoingImpl::getBaseLink() const
{
    return _base_link;
}

const std::string& VisualServoingImpl::getDistalLink() const
{
    return _distal_link;
}

const std::string& VisualServoingImpl::getFeatureType() const
{
    return _feature_type;
}


VisualServoingRos::VisualServoingRos(TaskDescription::Ptr task, RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_vs = std::dynamic_pointer_cast<VisualServoingTask>(task);
    if(!_ci_vs) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'VisualServoingTask'");


    /* Register type name */
    registerType("VisualServoing");
}

void VisualServoingRos::run(ros::Time time)
{
    TaskRos::run(time);
}

VisualServoingRosClient::VisualServoingRosClient(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh)
{

}

OpenSotVisualServoingAdapter::OpenSotVisualServoingAdapter(TaskDescription::Ptr ci_task,
                                         Context::ConstPtr context):
    OpenSotTaskAdapter(ci_task, context)
{
    _ci_vs = std::dynamic_pointer_cast<VisualServoingTask>(ci_task);
    if(!_ci_vs) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'VisualServoingTask'");
}

TaskPtr OpenSotVisualServoingAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);


    std::list<vpBasicFeature*> features;

    std::string id = "visual_servoing_"+_ci_vs->getBaseLink();
    _opensot_vs = boost::make_shared<VSSoT>(id, q, const_cast<ModelInterface&>(*_model),
                                            _ci_vs->getBaseLink(), _ci_vs->getDistalLink(),
                                            features);

    return _opensot_vs;
}

void OpenSotVisualServoingAdapter::processSolution(const Eigen::VectorXd& solution)
{
    OpenSotTaskAdapter::processSolution(solution);
}

