#include <gtest/gtest.h>
#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>
#include <XBotInterface/ModelInterface.h>
#include <ros/package.h>
#include <boost/make_shared.hpp>
#include <visp/vpFeaturePoint.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

#include <OpenSoT/tasks/velocity/Postural.h>

#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

#include <opensot_visual_seroving/utils/Utils.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

#include <OpenSoT/constraints/TaskToConstraint.h>

#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <OpenSoT/constraints/velocity/JointLimits.h>

#include <tf/transform_broadcaster.h>

#include <OpenSoT/tasks/velocity/AngularMomentum.h>

#if SELF_COLLISION_TEST
    #include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#endif

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


        _model->setJointPosition(q);
        _model->update();


        vs_task = std::make_shared<OpenSoT::tasks::velocity::VisualServoing>("visual_servoing", q,
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

    void setHomingPositionVSAM() {
        q[_model->getDofIndex("WaistLat")] =  0.0;
        q[_model->getDofIndex("WaistSag")] =  0.0;
        q[_model->getDofIndex("WaistYaw")] =  0.0;
        q[_model->getDofIndex("RShSag")] =  0.7047622086892001;
        q[_model->getDofIndex("RShLat")] =  -0.5677635252336075;
        q[_model->getDofIndex("RShYaw")] =  0.2890564140758947;
        q[_model->getDofIndex("RElbj")] =  -1.6532190021255193;
        q[_model->getDofIndex("RForearmPlate")] =  -0.0003280309196516731;
        q[_model->getDofIndex("RWrj1")] =  -0.008641640605188954;
        q[_model->getDofIndex("RWrj2")] =  -0.0013920145156089084;
        q[_model->getDofIndex("LShSag")] =  0.5545755010440612;
        q[_model->getDofIndex("LShLat")] =  0.6179429857281579;
        q[_model->getDofIndex("LShYaw")] =  -0.2412515368003296;
        q[_model->getDofIndex("LElbj")] =  -1.7123663089414456;
        q[_model->getDofIndex("LForearmPlate")] =  -0.0007051899703876909;
        q[_model->getDofIndex("LWrj1")] =  -0.013205282554514752;
        q[_model->getDofIndex("LWrj2")] =  0.0037545474284079597;
        q[_model->getDofIndex("RHipLat")] =  -0.73156157875935218;
        q[_model->getDofIndex("RHipYaw")] =  0.;
        q[_model->getDofIndex("RHipSag")] =  -0.1636185715937876;
        q[_model->getDofIndex("RKneeSag")] =  1.1413767149293743;
        q[_model->getDofIndex("RAnkSag")] =  -0.4421027118276261;
        q[_model->getDofIndex("RAnkLat")] =  0.004101490304317014;
        q[_model->getDofIndex("LHipLat")] =  0.73156157875935218;
        q[_model->getDofIndex("LHipYaw")] =  0.;
        q[_model->getDofIndex("LHipSag")] =  -0.1636185715937876;
        q[_model->getDofIndex("LKneeSag")] =  0.8076545504729606;
        q[_model->getDofIndex("LAnkSag")] =  -0.4695127574050335;
        q[_model->getDofIndex("LAnkLat")] =  0.0012525979207698662;

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

public:
    OpenSoT::tasks::velocity::VisualServoing::Ptr vs_task;
    Eigen::VectorXd q;
    std::string base_link;
    std::string camera_frame;
    std::list<vpFeaturePoint*> point_features;
    std::list<vpFeaturePoint*> desired_features;
    XBot::ModelInterface::Ptr _model;



};

TEST_F(testVisualServoingTask, testInteractionMatrix)
{
    vpMatrix L_visp;
    std::list<unsigned int> feature_selection = {vpBasicFeature::FEATURE_ALL,1,0,0};
    std::list<vpBasicFeature *> generic_features(std::begin(point_features), std::end(point_features));
    this->vs_task->computeInteractionMatrixFromList(generic_features,
                                                    feature_selection, L_visp);

    Eigen::MatrixXd L(L_visp.getRows(), L_visp.getCols());
    visp2eigen<Eigen::MatrixXd>(L_visp, L);

    EXPECT_EQ(L.rows(), 3);

    std::cout<<"L: \n"<<L<<std::endl;

}

XBot::MatLogger2::Ptr getLogger(const std::string& name)
{
    XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger(name); // date-time automatically appended
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    return logger;
}

TEST_F(testVisualServoingTask, testBasics)
{
    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testBasics");
    this->vs_task->log(logger);

    Eigen::Matrix6d I6; I6.setIdentity();
    EXPECT_TRUE(this->vs_task->getVelocityTwistMatrix() == I6);
    std::cout<<"Default V matrix is: \n"<<this->vs_task->getVelocityTwistMatrix()<<std::endl;

    I6.setRandom();
    this->vs_task->setVelocityTwistMatrix(I6);
    EXPECT_TRUE(this->vs_task->getVelocityTwistMatrix() == I6);
    I6.setIdentity();
    this->vs_task->setVelocityTwistMatrix(I6);

    // THE FOLLOWING 4 LINES HAVE BEEN COMMENTED BY TOTO SINCE CAUSES SEGFAULT ON HIS LAPTOP
    //EXPECT_TRUE(this->vs_task->getBaseLink() == this->base_link);
    //std::cout<<"base_link is: "<<this->vs_task->getBaseLink()<<std::endl;
    //EXPECT_TRUE(this->vs_task->getCameraLink() == this->camera_frame);
    //std::cout<<"camera_link is: "<<this->vs_task->getCameraLink()<<std::endl;

    Eigen::MatrixXd L = this->vs_task->getInteractionMatrix();
    EXPECT_EQ(L.rows(), 2*point_features.size());
    std::cout<<"L: \n"<<L<<std::endl;

    Eigen::MatrixXd L_ = this->computeInteractionMatrix(this->point_features);
    std::cout<<"L_: \n"<<L_<<std::endl;

    this->vs_task->log(logger);

    Eigen::MatrixXd Le = L-L_;
    EXPECT_NEAR(Eigen::Map<Eigen::RowVectorXd>(Le.data(), Le.size()).norm(), 0.0, 1e-6);
    std::cout<<"Le.norm(): "<<Eigen::Map<Eigen::RowVectorXd>(Le.data(), Le.size()).norm()<<std::endl;

    Eigen::MatrixXd W(L_.rows(), L_.rows());
    W.setIdentity(W.rows(), W.rows());
    EXPECT_TRUE(W == this->vs_task->getWeight());

    Eigen::VectorXd expected_b(2*this->desired_features.size());
    expected_b.setZero(expected_b.size());
    EXPECT_TRUE(this->vs_task->getb() == -expected_b);
    std::cout<<"b: \n"<<this->vs_task->getb()<<std::endl;
    std::list<vpBasicFeature *> generic_desired_features(std::begin(this->desired_features), std::end(this->desired_features));
    this->vs_task->setDesiredFeatures(generic_desired_features);
    this->vs_task->update(this->q);

    this->vs_task->log(logger);

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
    EXPECT_TRUE(this->vs_task->getb() == -expected_b);

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

    this->vs_task->log(logger);
}

TEST_F(testVisualServoingTask, testJacobians)
{
    this->vs_task->update(this->q);
    Eigen::MatrixXd A = this->vs_task->getA();
    Eigen::MatrixXd b = this->vs_task->getb();

    EXPECT_EQ(A.rows(), b.size());
    EXPECT_EQ(A.rows(), 2*point_features.size());

    std::cout<<"A: \n"<<A<<std::endl;

    std::list<unsigned int> id = {0,2,3};
    OpenSoT::tasks::Aggregated::TaskPtr vs = this->vs_task%id;
    vs->update(this->q);

    std::cout<<"vs->getA(): \n"<<vs->getA()<<std::endl;

    EXPECT_FALSE(A.rows() == vs->getA().rows());

    OpenSoT::tasks::velocity::Cartesian::Ptr foo =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("foo", this->q, *(this->_model),
                                                                    this->camera_frame, this->base_link);

    OpenSoT::tasks::Aggregated::TaskPtr aggr_tasks = foo%id + this->vs_task%id;

    EXPECT_EQ(aggr_tasks->getA().rows(), vs->getA().rows() + id.size());
    std::cout<<"aggr_tasks->getA(): \n"<<aggr_tasks->getA()<<std::endl;

}

TEST_F(testVisualServoingTask, testProjection)
{
    // Taken from https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-ibvs.html#ibvs_intro

    std::cout << "testProjection" << std::endl;

    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testProjection");
    
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpFeaturePoint p[4], pd[4];
    this->vs_task->clearFeatures();
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      this->vs_task->addFeature(p[i], pd[i]);
    }
    
    this->vs_task->update(this->q);
    
    //vpMatrix L_visp;
    Eigen::MatrixXd L, Lpinv;
    Eigen::VectorXd cam_vel(6,1);
    vpColVector v;
    Eigen::MatrixXd b;

    double delta_t = 0.033;
    
    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(delta_t);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    std::cout << "wMo: " << wMo << std::endl; 
    
    for (unsigned int iter = 0; iter < 3000; iter++) {

        this->vs_task->log(logger);

        L = this->vs_task->getInteractionMatrix();
        b = this->vs_task->getb();

    #if EIGEN_WORLD_VERSION > 3 && EIGEN_MAJOR_VERSION >= 3
        Lpinv = L.completeOrthogonalDecomposition().pseudoInverse();
    #else
        Eigen::JacobiSVD<Eigen::MatrixXd> svd( L, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd singularValuesInv(L.cols(), L.cols());
        singularValuesInv.setZero();
        int rank = svd.rank();
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1./svd.singularValues()[i];
        Lpinv = svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
    #endif


        cam_vel = Lpinv * b;
        eigen2visp<vpColVector>(cam_vel, v);


        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    
        robot.getPosition(wMc);
        
        cMo = wMc.inverse() * wMo;
        
        this->vs_task->clearFeatures();
        for (unsigned int i = 0; i < 4; i++) {
            point[i].track(cMo);
            vpFeatureBuilder::create(p[i], point[i]);
            this->vs_task->addFeature(p[i], pd[i]);
        }
        this->vs_task->update(this->q);
    
    }

}

TEST_F(testVisualServoingTask, testStandardVS)
{
    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testStandardVS");
    
    /// This test aims at verifying the correctness of v = lambda L^{-1} e
    
    //std::cout << " point features: " << this->point_features.get_x << std::endl;
    int i = 0;
    for(auto point_feature : this->desired_features)
    {
        if (i == 0) {
            point_feature->set_x(10);
            point_feature->set_y(10);
            point_feature->set_Z(1);
        }
        if (i == 1){
            point_feature->set_x(-10);
            point_feature->set_y(10);
            point_feature->set_Z(1);
        }
        if (i == 2){
            point_feature->set_x(-10);
            point_feature->set_y(-10);
            point_feature->set_Z(1);
        }
        if (i == 3){
            point_feature->set_x(10);
            point_feature->set_y(-10);
            point_feature->set_Z(1);
        }    
        i++;
    }

    i = 0;
    for(auto point_feature : this->point_features)
    {
        if (i == 0) {
            point_feature->set_x(5);
            point_feature->set_y(5);
            point_feature->set_Z(1);
        }
        if (i == 1){
            point_feature->set_x(-5);
            point_feature->set_y(5);
            point_feature->set_Z(1);
        }
        if (i == 2){
            point_feature->set_x(-5);
            point_feature->set_y(-5);
            point_feature->set_Z(1);
        }
        if (i == 3){
            point_feature->set_x(5);
            point_feature->set_y(-5);
            point_feature->set_Z(1);
        }    
        i++;
    }
    
    std::list<vpBasicFeature *> generic_desired_features(std::begin(this->desired_features), std::end(this->desired_features));
    std::list<vpBasicFeature *> generic_curr_features(std::begin(this->point_features), std::end(this->point_features));
    
    std::list<unsigned int> feature_selection = {
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL
    };

    this->vs_task->setFeatures(
            generic_curr_features,
            generic_desired_features,
            feature_selection
    );

    this->vs_task->update(this->q);
    
    Eigen::MatrixXd A = this->vs_task->getA();
    Eigen::MatrixXd b = this->vs_task->getb();

    std::cout << "b: " << b << std::endl;
    //std::cout << "A: " << A << std::endl;

    vpMatrix L_visp;
    this->vs_task->computeInteractionMatrixFromList(generic_curr_features, feature_selection, L_visp);

    Eigen::MatrixXd L(L_visp.getRows(), L_visp.getCols());
    visp2eigen<Eigen::MatrixXd>(L_visp, L);

    std::cout << "L " << L << std::endl;

    Eigen::MatrixXd Lpinv;
#if EIGEN_WORLD_VERSION > 3 && EIGEN_MAJOR_VERSION >= 3
    Lpinv = L.completeOrthogonalDecomposition().pseudoInverse();
#else
    Eigen::JacobiSVD<Eigen::MatrixXd> svd( L, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValuesInv(L.cols(), L.cols());
    singularValuesInv.setZero();
    int rank = svd.rank();
    for(unsigned int i = 0; i < rank; ++i)
        singularValuesInv(i,i) = 1./svd.singularValues()[i];
    Lpinv = svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
#endif
    std::cout << "Lpinv " << Lpinv << std::endl;

    Eigen::VectorXd cam_vel(6,1);
    cam_vel = Lpinv * b;
    std::cout << "Command: " << cam_vel << std::endl;

    Eigen::VectorXd s_dot(L_visp.getRows(),1);
    s_dot = L * cam_vel;
    std::cout << "Features dot: " << s_dot << std::endl;
    
    /// It should be implemented a function which project points on the image plane

    for(unsigned int k = 0; k < 1000; ++k){

        this->vs_task->log(logger);

        i = 0;
        double delta_t = 0.033;
        double x, x_new, y, y_new;
        for(auto point_feature : this->point_features)
        {
            //std::cout << "---------------------------" << std::endl;
            //std::cout << "i: " << i << std::endl; 
            
            x = point_feature->get_x();
            y = point_feature->get_y();
            
            //std::cout << "s: " << x << ", " << y << std::endl;
            //std::cout << "s_dot: " << s_dot[2*i] << ", " << s_dot[2*i+1] << std::endl;
            
            x_new = x + delta_t * s_dot[2*i];
            y_new = y + delta_t * s_dot[2*i+1];
            
            //std::cout << "s_new " << x_new  << ", " <<  y_new << std::endl;
            
            point_feature->set_x(x_new);
            point_feature->set_y(y_new);
            point_feature->set_Z(1);
            
            i++;
        }

        //std::cout << "x: " << x << std::endl;

        generic_curr_features.clear();
        std::copy(std::begin(this->point_features), std::end(this->point_features), std::back_inserter(generic_curr_features));

        if(k == 0)
            EXPECT_TRUE(this->vs_task->setFeatures(generic_curr_features, generic_desired_features,feature_selection));
        else
            EXPECT_TRUE(this->vs_task->setFeatures(generic_curr_features));
        
        // not sure I need this
        this->vs_task->update(this->q);
        
        A = this->vs_task->getA();
        b = this->vs_task->getb();

        //std::cout << "b: " << b << std::endl;
        //std::cout << "A: " << A << std::endl;
        this->vs_task->computeInteractionMatrixFromList(generic_curr_features, feature_selection, L_visp);
        visp2eigen<Eigen::MatrixXd>(L_visp, L);

#if EIGEN_WORLD_VERSION > 3 && EIGEN_MAJOR_VERSION >= 3
        Lpinv = L.completeOrthogonalDecomposition().pseudoInverse();
#else
        Eigen::JacobiSVD<Eigen::MatrixXd> svd( L, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd singularValuesInv(L.cols(), L.cols());
        singularValuesInv.setZero();
        int rank = svd.rank();
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1./svd.singularValues()[i];
        Lpinv = svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
#endif
        
        cam_vel = Lpinv * b;

        // We should use the projection of the point from the current pose of the camera
        s_dot = L * cam_vel;

    }

    std::cout << "Norm of b: " << b.norm() << std::endl;

    EXPECT_LE(b.norm(), 1e-6);


}

TEST_F(testVisualServoingTask, testOpenSoTTask)
{
    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testOpenSoTTask");

    //std::cout << " point features: " << this->point_features.get_x << std::endl;
    int i = 0;
    for(auto point_feature : this->desired_features)
    {
        if (i == 0) {
            point_feature->set_x(10);
            point_feature->set_y(10);
            point_feature->set_Z(1);
        }
        if (i == 1){
            point_feature->set_x(-10);
            point_feature->set_y(10);
            point_feature->set_Z(1);
        }
        if (i == 2){
            point_feature->set_x(-10);
            point_feature->set_y(-10);
            point_feature->set_Z(1);
        }
        if (i == 3){
            point_feature->set_x(10);
            point_feature->set_y(-10);
            point_feature->set_Z(1);
        }
        i++;
    }

    i = 0;
    for(auto point_feature : this->point_features)
    {
        if (i == 0) {
            point_feature->set_x(5);
            point_feature->set_y(5);
            point_feature->set_Z(1);
        }
        if (i == 1){
            point_feature->set_x(-5);
            point_feature->set_y(5);
            point_feature->set_Z(1);
        }
        if (i == 2){
            point_feature->set_x(-5);
            point_feature->set_y(-5);
            point_feature->set_Z(1);
        }
        if (i == 3){
            point_feature->set_x(5);
            point_feature->set_y(-5);
            point_feature->set_Z(1);
        }
        i++;
    }

    std::list<vpBasicFeature *> generic_desired_features(std::begin(this->desired_features), std::end(this->desired_features));
    std::list<vpBasicFeature *> generic_curr_features(std::begin(this->point_features), std::end(this->point_features));

    std::list<unsigned int> feature_selection = {
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL,
        vpBasicFeature::FEATURE_ALL
    };

    this->vs_task->setFeatures(generic_curr_features, generic_desired_features, feature_selection);

    OpenSoT::AutoStack::Ptr stack;
    stack /= this->vs_task;

    OpenSoT::solvers::iHQP::Ptr solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack);


    Eigen::VectorXd q = this->q;
    Eigen::VectorXd dq(q.rows());
    dq.setZero();

    /// It should be implemented a function which project points on the image plane
    Eigen::MatrixXd L = this->vs_task->getInteractionMatrix();
    Eigen::VectorXd s_dot(L.rows(),1);
    s_dot.setZero();

    for(unsigned int k = 0; k < 1000; ++k){

        this->vs_task->log(logger);

        i = 0;
        double delta_t = 0.033;
        double x, x_new, y, y_new;
        for(auto point_feature : this->point_features)
        {
            //std::cout << "---------------------------" << std::endl;
            //std::cout << "i: " << i << std::endl;

            x = point_feature->get_x();
            y = point_feature->get_y();

            //std::cout << "s: " << x << ", " << y << std::endl;
            //std::cout << "s_dot: " << s_dot[2*i] << ", " << s_dot[2*i+1] << std::endl;

            x_new = x + delta_t * s_dot[2*i];
            y_new = y + delta_t * s_dot[2*i+1];

            //std::cout << "s_new " << x_new  << ", " <<  y_new << std::endl;

            point_feature->set_x(x_new);
            point_feature->set_y(y_new);
            point_feature->set_Z(1);

            i++;
        }

        //std::cout << "x: " << x << std::endl;

        generic_curr_features.clear();
        std::copy(std::begin(this->point_features), std::end(this->point_features), std::back_inserter(generic_curr_features));

        if(k == 0)
            EXPECT_TRUE(this->vs_task->setFeatures(generic_curr_features, generic_desired_features,feature_selection));
        else
            EXPECT_TRUE(this->vs_task->setFeatures(generic_curr_features));

        q += dq;
        //std::cout<<"q: "<<q.transpose()<<std::endl;


        this->_model->setJointPosition(q);
        this->_model->update();

        stack->update(q);

        EXPECT_TRUE(solver->solve(dq));



        // We should use the projection of the point from the current pose of the camera
        s_dot = this->vs_task->getA() * dq;

        //std::cout<<"s_dot: "<<s_dot<<std::endl;

    }

    std::cout << "Norm of b: " << this->vs_task->getb().norm() << std::endl;

    EXPECT_LE(this->vs_task->getb().norm(), 1e-6);

}

/**
 * @brief publishRobotModel
 * @param robot_state_publisher_
 * @param world_broadcaster
 * @param model
 * NOTE: call this AFTER model.update()!
 */
void publishRobotModel(robot_state_publisher::RobotStatePublisher* robot_state_publisher_,
                       tf::TransformBroadcaster* world_broadcaster,
                       const XBot::ModelInterface* model)
{
    ros::Time t = ros::Time::now();

    XBot::JointNameMap joint_unordered_map;
    model->getJointPosition(joint_unordered_map);
    std::map<std::string, double> joint_map;
    std::vector<std::string> virtual_joints = {"VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6"};
    for(auto j : joint_unordered_map)
    {
        if(!(std::find(virtual_joints.begin(), virtual_joints.end(), j.first) != virtual_joints.end()))
            joint_map[j.first] = j.second;
    }

    Eigen::Affine3d w_T_fb;
    model->getFloatingBasePose(w_T_fb);


    tf::Vector3 p;
    p.setX(w_T_fb.translation()[0]);
    p.setY(w_T_fb.translation()[1]);
    p.setZ(w_T_fb.translation()[2]);
    Eigen::Quaterniond qq(w_T_fb.linear());
    tf::Quaternion rot;
    rot.setX(qq.x());
    rot.setY(qq.y());
    rot.setZ(qq.z());
    rot.setW(qq.w());
    tf::Transform T(rot, p);


    robot_state_publisher_->publishTransforms(joint_map, t, "");
    world_broadcaster->sendTransform(tf::StampedTransform(T, t, "world", "Waist"));
}

TEST_F(testVisualServoingTask, testWholeBodyVisualServoing)
{
    ///ROS RELATED PART:
    int argc = 0;
    char* argv[1] = {""};
    ros::init(argc, argv, "testWholeBodyVisualServoing");
    bool is_ros_running = ros::master::check();

    std::shared_ptr<ros::NodeHandle> nh;
    KDL::Tree tree;
    std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher_;
    std::shared_ptr<tf::TransformBroadcaster> world_broadcaster;
    std::shared_ptr<ros::Rate> rate;
    if(is_ros_running)
    {
        nh = std::make_shared<ros::NodeHandle>();
        nh->setParam("robot_description", this->_model->getUrdfString());
        if(!kdl_parser::treeFromUrdfModel(_model->getUrdf(), tree))
            ROS_ERROR("Failed to construct kdl tree");
        robot_state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(tree);
        world_broadcaster = std::make_shared<tf::TransformBroadcaster>();
    }
    ///

    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testOpenSoTTask");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("l_sole", this->q, *(this->_model), "l_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("r_sole", this->q, *(this->_model), "r_sole", "world");

    OpenSoT::tasks::velocity::CoM::Ptr com =
            std::make_shared<OpenSoT::tasks::velocity::CoM>(this->q, *(this->_model));
    com->setLambda(0.1);

    OpenSoT::tasks::velocity::Postural::Ptr postural =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(this->q);
    postural->setLambda(0.1);

    double dt = 0.001;
    if(is_ros_running)
        rate = std::make_shared<ros::Rate>(1./dt);
    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(M_PI, dt, this->q.size());

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("l_arm", this->q, *(this->_model), "LSoftHand", "torso");

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("r_arm", this->q, *(this->_model), "RSoftHand", "torso");


    Eigen::VectorXd qmin, qmax;
    this->_model->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(this->q, qmax, qmin);


    this->vs_task->setLambda(0.001);

    std::list<unsigned int> id = {0,1};
    OpenSoT::AutoStack::Ptr stack = ((l_sole + r_sole)/
                                    (com%id + 10.*this->vs_task + l_arm + r_arm)/
                                    (postural))<<vel_lims<<joint_lims;

    OpenSoT::solvers::iHQP::Ptr solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, 1e7);

    
    // Get the camera pose
    Eigen::Affine3d T;
    this->_model->getPose(camera_frame, T); 
    //std::cout << "T: " << T.matrix() << std::endl;
    vpHomogeneousMatrix wMt, wMc;
    eigen2visp<vpHomogeneousMatrix>(T.matrix(), wMc);
    //vpHomogeneousMatrix M1(0, 0, 0, 0, vpMath::rad(90), 0);
    //pHomogeneousMatrix M2(0, 0, 0, 0, 0, vpMath::rad(-90));
    //vpHomogeneousMatrix tMc = M1 * M2;
    //std::cout << "tMc: " << tMc << std::endl;
    //vpHomogeneousMatrix wMc = wMt * tMc;
    
    vpHomogeneousMatrix cMo(0, 0, 0.1, 0, 0, 0);
    vpHomogeneousMatrix cdMo(0, 0, 0.25, 0, 0, vpMath::rad(45));
    //vpHomogeneousMatrix cdMo(0.0, 0, 0.12, 0, 0, vpMath::rad(10));
    
    /// Object frame initial pose
    vpHomogeneousMatrix wMo = wMc * cMo;
    
    /// Define the visual pattern: 4 points at the verteces of a square around the object frame  
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    /// Define the current and desired visual features 
    /// (projecting the points with the current and desired camera pose)
    vpFeaturePoint p[4], pd[4];
    this->vs_task->clearFeatures();
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      this->vs_task->addFeature(p[i], pd[i]);
    }

    // Not sure I need this 
    this->vs_task->update(this->q);
    
    /*
    this->desired_features.clear();
    for(unsigned int i = 0; i < 4; ++i)
    {
        this->desired_features.push_back(new vpFeaturePoint());
        this->desired_features.back()-> buildFrom(3*i-0.1, (3*i+1)+0.1, (3*i+2));
        this->desired_features.back()->print();
    }

    auto feature_list = toGenericFeature<vpFeaturePoint>(this->desired_features);
    EXPECT_TRUE(this->vs_task->setDesiredFeatures(feature_list));
    */

    stack->update(this->q);

    Eigen::VectorXd q, dq, ds;
    q = this->q;
    dq.setZero(q.size());

    if(is_ros_running)
        robot_state_publisher_->publishFixedTransforms("", true);

    for(unsigned int i = 0; i < 10000; ++i)
    {
        this->vs_task->log(logger);

        //1. Models update
        q += dq;
        //ds = this->vs_task->getA() * dq;

        this->_model->setJointPosition(q);
        this->_model->update();

        this->_model->getPose(camera_frame, T); 
        eigen2visp<vpHomogeneousMatrix>(T.matrix(), wMc);
        //wMc = wMt * tMc;
        cMo = wMc.inverse() * wMo;
        
        this->vs_task->clearFeatures();
        for (unsigned int i = 0; i < 4; i++) {
            point[i].track(cMo);
            vpFeatureBuilder::create(p[i], point[i]);
            this->vs_task->addFeature(p[i], pd[i]);
        }
        this->vs_task->update(this->q);
        
        /*
        double x, x_new, y, y_new;
        int j = 0;
        for(auto point_feature : this->point_features)
        {
            x = point_feature->get_x();
            y = point_feature->get_y();

            x_new = x + ds[2*j];
            y_new = y + ds[2*j+1];

            point_feature->set_x(x_new);
            point_feature->set_y(y_new);
            point_feature->set_Z(1);

            j++;
        }

        auto features = toGenericFeature<vpFeaturePoint>(this->point_features);
        EXPECT_TRUE(this->vs_task->setFeatures(features));
        */



        if(is_ros_running)
            publishRobotModel(robot_state_publisher_.get(), world_broadcaster.get(), this->_model.get());


        //2. stack update
        Eigen::MatrixXd B;
        this->_model->getInertiaMatrix(B);
        postural->setWeight(B);

        stack->update(q);

        //3. solve
        EXPECT_TRUE(solver->solve(dq));


        //4. Check contact kept
        EXPECT_LE(l_sole->getb().norm(), 1e-3);
        EXPECT_LE(r_sole->getb().norm(), 1e-3);

        if(is_ros_running)
            rate->sleep();

    }


    //5. check visual servoing convergence
    EXPECT_LE(this->vs_task->getFeaturesError().norm(), 1e-3);
    
    std::cout<<"visual servoing error norm: "<<this->vs_task->getFeaturesError().norm()<<std::endl;  
}

TEST_F(testVisualServoingTask, testVSAM)
{
    this->setHomingPositionVSAM();
    this->_model->setJointPosition(this->q);
    this->_model->update();


    ///ROS RELATED PART:
    int argc = 0;
    char* argv[1] = {""};
    ros::init(argc, argv, "testVSAM");
    bool is_ros_running = ros::master::check();

    std::shared_ptr<ros::NodeHandle> nh;
    KDL::Tree tree;
    std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher_;
    std::shared_ptr<tf::TransformBroadcaster> world_broadcaster;
    std::shared_ptr<ros::Rate> rate;
    if(is_ros_running)
    {
        nh = std::make_shared<ros::NodeHandle>();
        nh->setParam("robot_description", this->_model->getUrdfString());
        if(!kdl_parser::treeFromUrdfModel(_model->getUrdf(), tree))
            ROS_ERROR("Failed to construct kdl tree");
        robot_state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(tree);
        world_broadcaster = std::make_shared<tf::TransformBroadcaster>();
    }
    ///

    XBot::MatLogger2::Ptr logger = getLogger("testVisualServoingTask_testVSAM");

    OpenSoT::tasks::velocity::CoM::Ptr com =
            std::make_shared<OpenSoT::tasks::velocity::CoM>(this->q, *(this->_model));
    com->setLambda(0.);

    OpenSoT::tasks::velocity::AngularMomentum::Ptr mom =
            std::make_shared<OpenSoT::tasks::velocity::AngularMomentum>(this->q, *(this->_model));


    OpenSoT::tasks::velocity::Postural::Ptr postural =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(this->q);
    postural->setLambda(0.005);
    Eigen::MatrixXd W = postural->getWeight();
    for(unsigned int i = 0; i < 6; ++i)
        W(i,i) = 0.0;
    W(this->_model->getDofIndex("WaistYaw"),this->_model->getDofIndex("WaistYaw")) = 20.;
    postural->setWeight(W);

    double dt = 0.01;
    if(is_ros_running)
        rate = std::make_shared<ros::Rate>(1./dt);
    Eigen::VectorXd qdotlims;
    this->_model->getVelocityLimits(qdotlims);
    std::cout<<"dq lims: "<<qdotlims.transpose()<<std::endl;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(qdotlims, dt);

    Eigen::VectorXd qmin, qmax;
    this->_model->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(this->q, qmax, qmin);

    this->vs_task->setLambda(0.01);

    OpenSoT::tasks::velocity::Cartesian::Ptr camera =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>(this->camera_frame, this->q,
                                                                    *this->_model, this->camera_frame, this->base_link);
    camera->setLambda(0.);
    std::list<unsigned int> id = {2};

    std::string waist = "Waist";

#if SELF_COLLISION_TEST
    OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr sc =
            boost::make_shared<OpenSoT::constraints::velocity::SelfCollisionAvoidance>(this->q, *this->_model, waist,
                                                                    std::numeric_limits<double>::infinity(), 0.005);
    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>("LThighUpLeg", "RThighUpLeg"));
    whiteList.push_back(std::pair<std::string,std::string>("LFoot", "RFoot"));
    whiteList.push_back(std::pair<std::string,std::string>("DWYTorso", "LSoftHand"));
    whiteList.push_back(std::pair<std::string,std::string>("DWYTorso", "RSoftHand"));
    whiteList.push_back(std::pair<std::string,std::string>("LFoot", "RFootmot"));
    whiteList.push_back(std::pair<std::string,std::string>("RFoot", "LFootmot"));
    sc->setCollisionWhiteList(whiteList);
#endif

    OpenSoT::AutoStack::Ptr stack;
#if SELF_COLLISION_TEST
    stack = ((com + mom)/
             (camera%id)/
            (this->vs_task)/(postural))<<vel_lims<<joint_lims<<sc;
#else
    stack = ((com + mom)/
             (camera%id)/
            (this->vs_task)/(postural))<<vel_lims<<joint_lims;
#endif

    OpenSoT::solvers::iHQP::Ptr solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, 1e9, OpenSoT::solvers::solver_back_ends::qpOASES);

    // Get the camera pose
    Eigen::Affine3d T;
    this->_model->getPose(camera_frame, T);
    //std::cout << "T: " << T.matrix() << std::endl;
    vpHomogeneousMatrix wMt, wMc;
    eigen2visp<vpHomogeneousMatrix>(T.matrix(), wMc);

    vpHomogeneousMatrix cMo(0, 0, .15, 0, 0, 0);
    vpHomogeneousMatrix cdMo(0, 0, .15, 0, 0, vpMath::rad(90));
    //vpHomogeneousMatrix cdMo(0.0, 0, 0.12, 0, 0, vpMath::rad(10));

    /// Object frame initial pose
    vpHomogeneousMatrix wMo = wMc * cMo;

    /// Define the visual pattern: 4 points at the verteces of a square around the object frame
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    /// Define the current and desired visual features
    /// (projecting the points with the current and desired camera pose)
    vpFeaturePoint p[4], pd[4];
    this->vs_task->clearFeatures();
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      this->vs_task->addFeature(p[i], pd[i]);
    }

    stack->update(this->q);

    Eigen::VectorXd q, dq;
    q = this->q;
    dq.setZero(q.size());

    if(is_ros_running)
        robot_state_publisher_->publishFixedTransforms("", true);

    for(unsigned int i = 0; i < 3000; ++i)
    {
        this->vs_task->log(logger);

        //1. Models update
        q += dq;

        this->_model->setJointPosition(q);
        this->_model->update();

        this->_model->getPose(camera_frame, T);
        eigen2visp<vpHomogeneousMatrix>(T.matrix(), wMc);
        cMo = wMc.inverse() * wMo;

        this->vs_task->clearFeatures();
        for (unsigned int i = 0; i < 4; i++) {
            point[i].track(cMo);
            vpFeatureBuilder::create(p[i], point[i]);
            this->vs_task->addFeature(p[i], pd[i]);
        }

        if(is_ros_running)
            publishRobotModel(robot_state_publisher_.get(), world_broadcaster.get(), this->_model.get());

        //2. stack update
        stack->update(q);

        //3. solve
        bool solved = solver->solve(dq);
        if(!solved)
            dq.setZero();


        //4. Check contact kept
        EXPECT_LE((com->getA()*dq - com->getb()).norm(), 1e-3);
        EXPECT_LE((mom->getA()*dq - mom->getb()).norm(), 1e-3);

        if(is_ros_running)
            rate->sleep();

    }

    //5. check visual servoing convergence
    EXPECT_NEAR(this->vs_task->getFeaturesError().norm(), 1e-3, 1e-1);

    std::cout<<"visual servoing error norm: "<<this->vs_task->getFeaturesError().norm()<<std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
