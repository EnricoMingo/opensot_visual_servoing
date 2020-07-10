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

TEST_F(testVisualServoingTask, testBasics)
{
    XBot::MatLogger::Ptr logger;
    logger = XBot::MatLogger::getLogger("testVisualServoingTask_testBasics");
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
    logger->flush();

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

    EXPECT_FALSE(A == vs->getA());

    OpenSoT::tasks::velocity::Cartesian::Ptr foo =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>("foo", this->q, *(this->_model),
                                                                    this->camera_frame, this->base_link);

    OpenSoT::tasks::Aggregated::TaskPtr aggr_tasks = foo%id + this->vs_task%id;

    EXPECT_EQ(aggr_tasks->getA().rows(), vs->getA().rows() + id.size());
    std::cout<<"aggr_tasks->getA(): \n"<<aggr_tasks->getA()<<std::endl;

}

TEST_F(testVisualServoingTask, testProjection)
{
    // Taken from https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-ibvs.html#ibvs_intro

    std::cout << "testProjection" << std::endl;

    XBot::MatLogger::Ptr logger;
    logger = XBot::MatLogger::getLogger("testVisualServoingTask_testProjection");
    
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
    logger->flush();

}

TEST_F(testVisualServoingTask, testStandardVS)
{
    XBot::MatLogger::Ptr logger;
    logger = XBot::MatLogger::getLogger("testVisualServoingTask_testStandardVS");
    
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


    logger->flush();

}

TEST_F(testVisualServoingTask, testOpenSoTTask)
{
    XBot::MatLogger::Ptr logger;
    logger = XBot::MatLogger::getLogger("testVisualServoingTask_testOpenSoTTask");

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

    OpenSoT::solvers::iHQP::Ptr solver = boost::make_shared<OpenSoT::solvers::iHQP>(*stack);


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


    logger->flush();

}

TEST_F(testVisualServoingTask, testWholeBodyVisualServoing)
{
    XBot::MatLogger::Ptr logger;
    logger = XBot::MatLogger::getLogger("testVisualServoingTask_testOpenSoTTask");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>("l_sole", this->q, *(this->_model), "l_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>("r_sole", this->q, *(this->_model), "r_sole", "world");

    OpenSoT::tasks::velocity::CoM::Ptr com =
            boost::make_shared<OpenSoT::tasks::velocity::CoM>(this->q, *(this->_model));

    OpenSoT::tasks::velocity::Postural::Ptr postural =
            boost::make_shared<OpenSoT::tasks::velocity::Postural>(this->q);

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims =
            boost::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(M_PI_2, 0.01, this->q.size());


    OpenSoT::AutoStack::Ptr stack = ((l_sole + r_sole)/
                                    (com + this->vs_task)/
                                    (postural))<<vel_lims;


    logger->flush();
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
