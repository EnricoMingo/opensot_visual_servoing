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

        for(unsigned int i = 0; i < 4; ++i)
        {
            point_features.push_back(new vpFeaturePoint());
            point_features.back()-> buildFrom(3*i, 3*i+1, 3*i+2);
            point_features.back()->print();

            desired_features.push_back(new vpFeaturePoint());
            desired_features.back()-> buildFrom(2*3*i, 2*(3*i+1), 2*(3*i+2));
            desired_features.back()->print();
        }



        std::list<vpBasicFeature *> generic_features(std::begin(point_features), std::end(point_features));


        vs_task = boost::make_shared<OpenSoT::tasks::velocity::VisualServoing>("visual_servoing", q,
                                                                                *_model,
                                                                                base_link, camera_frame,
                                                                                generic_features);

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

    Eigen::MatrixXd computeInteractionMatrix(std::list<vpFeaturePoint*>& point_features)
    {
        Eigen::MatrixXd L(2*point_features.size(), 6);
        L.setZero(L.rows(), L.cols());
        int j = 0;
        for(auto feature : point_features)
        {
            double x = feature->get_x();
            double y = feature->get_y();
            double Z = feature->get_Z();

            L.row(j) <<  -1./Z,   0., x/Z, x*y, -(1.+x*x), y;
            L.row(j+1) << 0., -1./Z, y/Z, 1.+y*y, -x*y , -x;
            j += 2;
        }
        return L;
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
    std::list<vpFeaturePoint*> point_features;
    std::list<vpFeaturePoint*> desired_features;



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


    EXPECT_TRUE(this->vs_task->getBaseLink() == this->base_link);
    std::cout<<"base_link is: "<<this->vs_task->getBaseLink()<<std::endl;
    EXPECT_TRUE(this->vs_task->getCameraLink() == this->camera_frame);
    std::cout<<"camera_link is: "<<this->vs_task->getCameraLink()<<std::endl;


    Eigen::MatrixXd L = this->vs_task->getInteractionMatrix();
    EXPECT_EQ(L.rows(), 2*point_features.size());
    std::cout<<"L: \n"<<L<<std::endl;

    Eigen::MatrixXd L_ = this->computeInteractionMatrix(this->point_features);
    std::cout<<"L_: \n"<<L_<<std::endl;

    Eigen::MatrixXd Le = L-L_;
    EXPECT_NEAR(Eigen::Map<Eigen::RowVectorXd>(Le.data(), Le.size()).norm(), 0.0, 1e-6);
    std::cout<<"Le.norm(): "<<Eigen::Map<Eigen::RowVectorXd>(Le.data(), Le.size()).norm()<<std::endl;

    Eigen::MatrixXd W(L_.rows(), L_.rows());
    W.setIdentity(W.rows(), W.rows());
    EXPECT_TRUE(W == this->vs_task->getWeight());

    Eigen::VectorXd expected_b(2*this->desired_features.size());
    expected_b.setZero(expected_b.size());
    EXPECT_TRUE(this->vs_task->getb() == expected_b);
    std::cout<<"b: \n"<<this->vs_task->getb()<<std::endl;
    std::list<vpBasicFeature *> generic_desired_features(std::begin(this->desired_features), std::end(this->desired_features));
    this->vs_task->setDesiredFeatures(generic_desired_features);
    this->vs_task->update(this->q);
    std::list<vpFeaturePoint*>::iterator p = this->desired_features.begin();
    unsigned int i = 0;
    for(auto point_feature : this->point_features)
    {
        vpColVector error = point_feature->error(*(*p));
        expected_b[i] = error[0];
        expected_b[i+1] = error[1];
        i+=2;
        p++;
    }
    std::cout<<"expected_b: \n"<<expected_b<<std::endl;
    std::cout<<"b: \n"<<this->vs_task->getb()<<std::endl;
    EXPECT_TRUE(this->vs_task->getb() == expected_b);

    i = 0;
    for(auto point_feature : this->point_features)
    {
        point_feature->set_x(2*3*i);
        point_feature->set_y(2*(3*i+1));
        point_feature->set_Z(2*(3*i+2));
        i++;
    }
    this->vs_task->update(this->q);
    expected_b.setZero(expected_b.size());
    std::cout<<"expected_b: \n"<<expected_b<<std::endl;
    std::cout<<"b: \n"<<this->vs_task->getb()<<std::endl;
    EXPECT_TRUE(this->vs_task->getb() == expected_b);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
