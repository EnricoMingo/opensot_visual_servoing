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

void VisualServoingImpl::setVelocityTwistMatrix(const Eigen::Matrix6d& V)
{
    _V = V;
}

void VisualServoingImpl::addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select)
{
    _featureList.push_back(&s_cur);
    _desiredFeatureList.push_back(&s_star);
    _featureSelectionList.push_back(select);
}

bool VisualServoingImpl::setFeatures(std::list<vpBasicFeature *>& feature_list,
                                     std::list<vpBasicFeature *>& desired_feature_list,
                                     std::list<unsigned int>& feature_selection_list)
{
    _featureList = feature_list;
    _desiredFeatureList = desired_feature_list;
    _featureSelectionList = feature_selection_list;
}

bool VisualServoingImpl::setFeatures(std::list<vpBasicFeature *>& feature_list)
{
    _featureList = feature_list;
}

bool VisualServoingImpl::setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list)
{
    _desiredFeatureList = desired_feature_list;
}

/** **/
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

