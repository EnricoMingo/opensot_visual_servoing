#include "VisualServoing.h"
#include <visp/vpFeaturePoint.h>
#include <visp/vpGenericFeature.h>
#include <opensot_visual_seroving/utils/Utils.h>

using namespace XBot::Cartesian::velocity;

namespace  {

class dummy_feature : public vpBasicFeature
{
public:
    dummy_feature(unsigned int size):
        vpBasicFeature()
    {
        _size = size;
        vpColVector s(size, 0.);
        this->s = s;
    }

    vpMatrix interaction(const unsigned int select = FEATURE_ALL)
    {
        vpMatrix L(_size, 6, 0.);
        int r = _size;
        if(r > 6)
            r = 6;
        for(unsigned int i = 0; i < r; ++i)
            L[i][i] = 1.;
        return L;
    }

    void init()
    {
        /* do nothing*/
    }

    void print(const unsigned int select= FEATURE_ALL) const
    {
        /* do nothing*/
    }

    dummy_feature *duplicate() const
    {
        dummy_feature *feature= new dummy_feature(_size) ;

        return feature ;
    }

    void display(const vpCameraParameters &cam,
                 const vpImage<unsigned char> &I,
                 const vpColor &color=vpColor::green,
                 unsigned int thickness=1) const
    {
        /* do nothing*/
    }

    void display(const vpCameraParameters &cam,
                         const vpImage<vpRGBa> &I,
                         const vpColor &color=vpColor::green,
                         unsigned int thickness=1) const
    {
        /* do nothing*/
    }

private:
    unsigned int _size;
};

std::string get_name(YAML::Node node)
{
    auto distal_link = node["distal_link"].as<std::string>();
    return "visual_servoing_" + distal_link;
}

int get_size(YAML::Node node)
{
    if(!node["feature_type"])
    {
        std::string error = "feature_type parameter mandatory in VisualServoing task!\n";
        error += "Possible feature types are: \n";
        for(auto ft : FEATURE_TYPE)
            error += ft.first + "\n";
        throw std::runtime_error(error);
    }
    std::string feature_type = node["feature_type"].as<std::string>();

    if(!node["number_of_features"])
        throw std::runtime_error("number_of_features parameter mandatory in VisualServoing task!");

    int number_of_features = node["number_of_features"].as<int>();
    return FEATURE_TYPE.at(feature_type)*number_of_features;
}

}


VisualServoingImpl::VisualServoingImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, get_name(node), get_size(node))
{
    if(node["base_link"])
        _base_link = node["base_link"].as<std::string>();
    else
        _base_link = "world";

    if(!node["distal_link"])
        throw std::runtime_error("distal_link parameter mandatory in VisualServoing task!");
    _distal_link = node["distal_link"].as<std::string>();

    if(!node["feature_type"])
    {
        std::string error = "feature_type parameter mandatory in VisualServoing task!\n";
        error += "Possible feature types are: \n";
        for(auto ft : FEATURE_TYPE)
            error += ft.first + "\n";
        throw std::runtime_error(error);
    }
    _feature_type = node["feature_type"].as<std::string>();

    dummy_feature f(get_size(node));
    _featureList.push_back(f.duplicate());
    _desiredFeatureList.push_back(f.duplicate());
    _featureSelectionList.push_back(vpBasicFeature::FEATURE_ALL);

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

const std::list<vpBasicFeature * >& VisualServoingImpl::getFeatures() const
{
    return _featureList;
}

const std::list<vpBasicFeature * >& VisualServoingImpl::getDesiredFeatures() const
{
    return _desiredFeatureList;
}

const std::list<unsigned int>& VisualServoingImpl::getFeatureSelectionList() const
{
    return _featureSelectionList;
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

    return true;
}

bool VisualServoingImpl::setFeatures(std::list<vpBasicFeature *>& feature_list)
{
    _featureList = feature_list;
    if(_featureSelectionList.size() == 0)
    {
        for(unsigned int i = 0; i < _featureList.size(); ++i)
            _featureSelectionList.push_back(vpBasicFeature::FEATURE_ALL);
    }
    return true;
}

bool VisualServoingImpl::setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list)
{
    _desiredFeatureList = desired_feature_list;
    return true;
}

/** **/
std::list<vpBasicFeature*> VisualServoingRos::getFeaturesFromMsg(opensot_visual_servoing::VisualFeaturesConstPtr msg)
{
    std::list<vpBasicFeature *> generic_features;

    std::list<vpFeaturePoint *> point_features;
    for(unsigned int i = 0; i < msg->features.size(); ++i)
    {
        opensot_visual_servoing::VisualFeature f = msg->features[i];
        if(f.type == opensot_visual_servoing::VisualFeature::POINT)
        {
            point_features.push_back(new vpFeaturePoint());
            point_features.back()->buildFrom(f.x, f.y, f.Z);
         }
        //else if (f.type == opensot_visual_servoing::VisualFeature::LINE) ...
    }

    for(auto pf : point_features)
        generic_features.push_back(pf);
    return generic_features;
}

VisualServoingRos::VisualServoingRos(TaskDescription::Ptr task, RosContext::Ptr context):
    TaskRos(task, context), _visual_servoing_init(false)
{
    _ci_vs = std::dynamic_pointer_cast<VisualServoingTask>(task);
    if(!_ci_vs) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'VisualServoingTask'");

    auto on_features_recv = [this](opensot_visual_servoing::VisualFeaturesConstPtr msg)
    {
        ///TODO: HERE WE SHOULD STORE A NUMBER OF FEATURES SPECIFIED IN THE CONFIG
        std::list<vpBasicFeature*> generic_features = getFeaturesFromMsg(msg);

        if(!_visual_servoing_init)
        {
            std::list<unsigned int> feature_selection_list;
            for(unsigned int i = 0; i < generic_features.size(); ++i)
                feature_selection_list.push_back(vpBasicFeature::FEATURE_ALL);

            _ci_vs->setFeatures(generic_features, generic_features, feature_selection_list);
            _visual_servoing_init = true;
        }
        else
            _ci_vs->setFeatures(generic_features);
    };

    _feature_sub = _ctx->nh().subscribe<opensot_visual_servoing::VisualFeatures>(
                task->getName() + "/features", 10, on_features_recv);

    auto on_desired_features_recv = [this](opensot_visual_servoing::VisualFeaturesConstPtr msg)
    {
        std::list<vpBasicFeature*> generic_features = getFeaturesFromMsg(msg);
        _ci_vs->setDesiredFeatures(generic_features);
        if(!_visual_servoing_init)
            _visual_servoing_init = true;
    };

    _desired_feature_sub = _ctx->nh().subscribe<opensot_visual_servoing::VisualFeatures>(
                task->getName() + "/desired_features", 10, on_desired_features_recv);

    _reference_features = _ctx->nh().advertise<opensot_visual_servoing::VisualFeatures>(
                task->getName() + "/reference_features", 10, true);


    /* Register type name */
    registerType("VisualServoing");
}

opensot_visual_servoing::VisualFeatures
VisualServoingRos::toVisualFeatureMsg(const std::list<vpBasicFeature * >& feature_list,
                                     const std::string& features_type)
{
    opensot_visual_servoing::VisualFeatures msg;
    opensot_visual_servoing::VisualFeature feature;

    if(features_type == "vpFeaturePoint")
    {
        for(auto f : feature_list)
        {
           vpFeaturePoint * p = (vpFeaturePoint*)(f);
           feature.type = opensot_visual_servoing::VisualFeature::POINT;
           feature.x = p->get_x();
           feature.y = p->get_y();
           feature.Z = p->get_Z(); //
           msg.features.push_back(feature);
        }
    }

    msg.header.stamp = ros::Time::now();

    return msg;
}


void VisualServoingRos::run(ros::Time time)
{
    TaskRos::run(time);

    _reference_features.publish(toVisualFeatureMsg(_ci_vs->getDesiredFeatures(), _ci_vs->getFeatureType()));
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
    std::list<vpBasicFeature *> features = _ci_vs->getFeatures();

    std::string id = "visual_servoing_"+_ci_vs->getDistalLink();
    _opensot_vs = std::make_shared<VSSoT>(id,const_cast<ModelInterface&>(*_model),
                                            _ci_vs->getBaseLink(), _ci_vs->getDistalLink(), features);
    _opensot_vs->setLambda(_ci_vs->getLambda());

    return _opensot_vs;
}

void OpenSotVisualServoingAdapter::processSolution(const Eigen::VectorXd& solution)
{
    OpenSotTaskAdapter::processSolution(solution);
}

void OpenSotVisualServoingAdapter::update(double time, double period)
{
    /* Update (desired) features */
    std::list<vpBasicFeature *> features = _ci_vs->getFeatures();
    std::list<vpBasicFeature *> desired_features = _ci_vs->getDesiredFeatures();
    std::list<unsigned int> feature_selection_list = _ci_vs->getFeatureSelectionList();
    _opensot_vs->setFeatures(features, desired_features, feature_selection_list);
    _opensot_vs->setLambda(_ci_vs->getLambda());
}

CARTESIO_REGISTER_TASK_PLUGIN(VisualServoingImpl, VisualServoing)
CARTESIO_REGISTER_ROS_API_PLUGIN(VisualServoingRos, VisualServoing)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotVisualServoingAdapter, VisualServoing)
