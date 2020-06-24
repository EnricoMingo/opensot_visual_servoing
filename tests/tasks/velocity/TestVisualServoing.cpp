#include <gtest/gtest.h>
#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>
#include <XBotInterface/ModelInterface.h>
#include <ros/package.h>
#include <boost/make_shared.hpp>
#include <visp/vpFeaturePoint.h>

namespace{

class testVisualServoingTask: public ::testing::Test
{
protected:
    testVisualServoingTask()
    {
        std::string coman_urdf_path = ros::package::getPath("coman_urdf")+"/urdf/coman.urdf";
        std::cout<<"coman_urdf_path: "<<coman_urdf_path<<std::endl;

        std::string coman_srdf_path = ros::package::getPath("coman_srdf")+"/srdf/coman.srdf";
        std::cout<<"coman_srdf_path: "<<coman_srdf_path<<std::endl;

        ///TODO: check if files exists!

        XBot::ConfigOptions opt;
        opt.set_urdf_path(coman_urdf_path);
        opt.set_srdf_path(coman_srdf_path);
        opt.set_parameter<bool>("is_model_floating_base", true);
        opt.set_parameter<std::string>("model_type", "RBDL");
        opt.generate_jidmap();
        _model = XBot::ModelInterface::getModel(opt);

        base_link = "world";
        camera_frame = "torso";

        q.setZero(_model->getJointNum());
        setHomingPosition();

        vs_task = boost::make_shared<OpenSoT::tasks::velocity::VisualServoing>("visual_servoing", q,
                                                                                *_model,
                                                                                base_link, camera_frame);
    }

    void setHomingPosition() {
        q[_model->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        q[_model->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        q[_model->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        q[_model->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        q[_model->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        q[_model->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        q[_model->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        q[_model->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        q[_model->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        q[_model->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        q[_model->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        q[_model->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        q[_model->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        q[_model->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    }

    virtual ~testVisualServoingTask()
    {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    XBot::ModelInterface::Ptr _model;

public:
    OpenSoT::tasks::velocity::VisualServoing::Ptr vs_task;
    Eigen::VectorXd q;
    std::string base_link;
    std::string camera_frame;



};

TEST_F(testVisualServoingTask, testBasics)
{
    Eigen::Matrix6d I6; I6.setIdentity();
    EXPECT_TRUE(this->vs_task->getVelocityTwistMatrix() == I6);
    std::cout<<"Default V matrix is: \n"<<this->vs_task->getVelocityTwistMatrix()<<std::endl;

    I6.setRandom();
    this->vs_task->setVelocityTwistMatrix(I6);
    EXPECT_TRUE(this->vs_task->getVelocityTwistMatrix() == I6);
    I6.setIdentity();
    this->vs_task->setVelocityTwistMatrix(I6);

    Eigen::MatrixXd i6(1,6); i6.setZero(1,6);
    EXPECT_TRUE(this->vs_task->getInteractionMatrix() == i6);
    std::cout<<"Default L matrix is: \n"<<this->vs_task->getInteractionMatrix()<<std::endl;

    EXPECT_TRUE(this->vs_task->getBaseLink() == this->base_link);
    std::cout<<"base_link is: "<<this->vs_task->getBaseLink()<<std::endl;
    EXPECT_TRUE(this->vs_task->getCameraLink() == this->camera_frame);
    std::cout<<"camera_link is: "<<this->vs_task->getCameraLink()<<std::endl;

    vpFeaturePoint point_feature;
    point_feature.buildFrom(1., 2., 1.);
    vpFeaturePoint desired_point_feature;
    point_feature.buildFrom(0., 0., 1.);
    this->vs_task->addFeature(point_feature, desired_point_feature);
    auto feature_list = this->vs_task->getFeatures();
    EXPECT_EQ(feature_list.size(), 1);
    auto desired_feature_list = this->vs_task->getDesiredFeatures();
    EXPECT_EQ(desired_feature_list.size(), 1);

   // std::cout<<"(*feature_list.begin())->get_s(): \n"<<(*feature_list.begin())->get_s()<<std::endl;
   // std::cout<<"(*desired_feature_list.begin())->get_s(): \n"<<(*desired_feature_list.begin())->get_s()<<std::endl;


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
