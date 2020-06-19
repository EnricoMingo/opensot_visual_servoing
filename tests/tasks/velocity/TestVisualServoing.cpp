#include <gtest/gtest.h>
#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>
#include <XBotInterface/ModelInterface.h>
#include <ros/package.h>

namespace{

class testVisualServoingTask: public ::testing::Test
{
protected:
    testVisualServoingTask()
    {
        std::string coman_urdf_path = ros::package::getPath("coman_urdf");
        std::cout<<"coman_urdf_path: "<<coman_urdf_path<<std::endl;


        //XBot::ConfigOptions opt;

        //_model = XBot::ModelInterface::getModel()
    }

    virtual ~testVisualServoingTask()
    {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    XBot::ModelInterface::Ptr _model;
};

TEST_F(testVisualServoingTask, testVisualServoingTask_)
{

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
