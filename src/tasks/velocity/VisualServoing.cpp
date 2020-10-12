#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>
#include <visp/vpServo.h>
#include <boost/make_shared.hpp>
#include <opensot_visual_seroving/utils/Utils.h>



using namespace OpenSoT::tasks::velocity;

VisualServoing::VisualServoing(std::string task_id,
                               const Eigen::VectorXd &x,
                               XBot::ModelInterface &robot,
                               std::string base_link,
                               std::string camera_link,
                               std::list<vpBasicFeature *>& feature_list):
    Task(task_id, x.size()),
    _featureList(feature_list),
    _desiredFeatureList(feature_list),
    _eye_in_hand(true)
{
    for(unsigned int i = 0; i < _featureList.size(); ++i)
        _featureSelectionList.push_back(vpBasicFeature::FEATURE_ALL);


    _cartesian_task = boost::make_shared<Cartesian>("vs_"+camera_link, x, robot, camera_link, base_link);
    _cartesian_task->setLambda(0.0); // not needed actually, just as reminder
    _cartesian_task->setIsBodyJacobian(true); //we want to control the robot in camera_frame

    _V.setIdentity(); //intialize sensor_frame and camera_frame coincident 

    if(feature_list.size() > 0)
    {
        computeInteractionMatrix(); //Here I am initializing _L
        _W.setIdentity(_L.rows(),_L.rows()); //initialized
    }
    else
    {
        _L.setIdentity(6,6);
        _W.setIdentity(6,6);
    }

    _hessianType = HST_SEMIDEF;
    update(x);
}

void VisualServoing::_update(const Eigen::VectorXd &x)
{

    //1) computes Cartesian quantities from Cartesian task
    _cartesian_task->update(x);
    _J = _cartesian_task->getA(); //body jacobian

    //2) computes new Jacobian using the interaction matrix from VISP
    Eigen::MatrixXd tmp = _J;
    tmp.noalias() = _V*_J;
    _A.noalias() = _L*tmp;

    if(_featureList.size() > 0)
    {
        if(!_eye_in_hand)
            _A *= -1.;

        //3) computes task error
        compute_b();
    }
    else
    {
        //3) computes task error
        _b.setZero(6);
    }

}

void VisualServoing::setVelocityTwistMatrix(const Eigen::Matrix6d& V)
{
    _V = V;
}

const Eigen::Matrix6d& VisualServoing::getVelocityTwistMatrix() const
{
    return _V;
}

void VisualServoing::computeInteractionMatrix()
{
    computeInteractionMatrixFromList(_featureList, _featureSelectionList, _L_visp);
    visp2eigen<Eigen::MatrixXd>(_L_visp, _L);

    if(_col_indices.size() > 0)
    {
        for(std::list<unsigned int>::iterator i = _col_indices.begin(); i != _col_indices.end(); i++)
            _L.col(*i).setZero();
    }
}

/**
 * @brief VisualServoing::computeInteractionMatrixFromList
 * Copied from https://visp-doc.inria.fr/doxygen/visp-daily/vpServo_8cpp_source.html#l00580
 */
void VisualServoing::computeInteractionMatrixFromList(const std::list<vpBasicFeature *> &featureList,
                                              const std::list<unsigned int> &featureSelectionList, vpMatrix &L)
{
   if (featureList.empty()) {
     vpERROR_TRACE("feature list empty, cannot compute Ls");
     throw(vpServoException(vpServoException::noFeatureError, "feature list empty, cannot compute Ls"));
   }

   /* The matrix dimension is not known before the affectation loop.
    * It thus should be allocated on the flight, in the loop.
    * The first assumption is that the size has not changed. A double
    * reallocation (realloc(dim*2)) is done if necessary. In particular,
    * [log_2(dim)+1] reallocations are done for the first matrix computation.
    * If the allocated size is too large, a correction is done after the loop.
    * The algorithmic cost is linear in affectation, logarithmic in allocation
    * numbers and linear in allocation size.
    */

   /* First assumption: matrix dimensions have not changed. If 0, they are
    * initialized to dim 1.*/
   unsigned int rowL = L.getRows();
   const unsigned int colL = 6;
   if (0 == rowL) {
     rowL = 1;
     L.resize(rowL, colL);
   }

   /* vectTmp is used to store the return values of functions get_s() and
    * error(). */
   vpMatrix matrixTmp;

   /* The cursor are the number of the next case of the vector array to
    * be affected. A memory reallocation should be done when cursor
    * is out of the vector-array range.*/
   unsigned int cursorL = 0;

   std::list<vpBasicFeature *>::const_iterator it;
   std::list<unsigned int>::const_iterator it_select;

   for (it = featureList.begin(), it_select = featureSelectionList.begin(); it != featureList.end(); ++it, ++it_select) {
     /* Get s. */
     matrixTmp = (*it)->interaction(*it_select);
     unsigned int rowMatrixTmp = matrixTmp.getRows();
     unsigned int colMatrixTmp = matrixTmp.getCols();

     /* Check the matrix L size, and realloc if needed. */
     while (rowMatrixTmp + cursorL > rowL) {
       rowL *= 2;
       L.resize(rowL, colL, false);
       vpDEBUG_TRACE(15, "Realloc!");
     }

     /* Copy the temporarily matrix into L. */
     for (unsigned int k = 0; k < rowMatrixTmp; ++k, ++cursorL) {
       for (unsigned int j = 0; j < colMatrixTmp; ++j) {
         L[cursorL][j] = matrixTmp[k][j];
       }
     }
   }

   L.resize(cursorL, colL, false);

   return;
}

void VisualServoing::clearFeatures()
{
    _featureList.clear();
    _desiredFeatureList.clear();
    _featureSelectionList.clear();
}

void VisualServoing::addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select)

{
  _featureList.push_back(&s_cur);
  _desiredFeatureList.push_back(&s_star);
  _featureSelectionList.push_back(select);

  computeInteractionMatrix();

  _W.setIdentity(_L.rows(),_L.rows());
}

bool VisualServoing::setFeatures(std::list<vpBasicFeature *>& feature_list,
                 std::list<vpBasicFeature *>& desired_feature_list,
                 std::list<unsigned int>& feature_selection_list)
{
    if(feature_list.size() != desired_feature_list.size())
    {
        std::cerr<<"feature_list.size() != desired_feature_list.size(): "<<feature_list.size()<<" != "<<desired_feature_list.size()<<std::endl;
        return false;
    }
    if(feature_selection_list.size() != desired_feature_list.size())
    {
        std::cerr<<"feature_selection_list.size() != desired_feature_list.size(): "<<feature_selection_list.size()<<" != "<<desired_feature_list.size()<<std::endl;
        return false;
    }

    _featureList = feature_list;
    _desiredFeatureList = desired_feature_list;
    _featureSelectionList = feature_selection_list;

    computeInteractionMatrix();
    _W.setIdentity(_L.rows(),_L.rows());

    return true;
}

bool VisualServoing::setFeatures(std::list<vpBasicFeature *>& feature_list)
{
    if(feature_list.size() != _featureList.size())
    {
        XBot::Logger::error("feature_list.size() is %i, should be %i", feature_list.size(), _featureList.size());
        return false;
    }

    _featureList = feature_list;
    computeInteractionMatrix();
    return true;
}

const std::list<vpBasicFeature *>& VisualServoing::getFeatures() const
{
    return _featureList;
}

const std::list<vpBasicFeature *>& VisualServoing::getDesiredFeatures() const
{
    return _desiredFeatureList;
}

const std::string& VisualServoing::getBaseLink() const
{
    return _cartesian_task->getBaseLink();
}

const std::string& VisualServoing::getCameraLink() const
{
    return _cartesian_task->getDistalLink();
}

const Eigen::MatrixXd& VisualServoing::getInteractionMatrix() const
{
    return _L;
}

void VisualServoing::setEyeInHand()
{
    _eye_in_hand = true;
}

bool VisualServoing::isEyeInHand()
{
    return _eye_in_hand;
}

void VisualServoing::setEyeToHand()
{
    _eye_in_hand = false;
}

bool VisualServoing::isEyeToHand()
{
    return !_eye_in_hand;
}

void VisualServoing::compute_b()
{
    _b.resize(_L.rows());
    _features_error.resize(_L.rows());
    std::list<vpBasicFeature*>::iterator desired_feature = _desiredFeatureList.begin();
    std::list<unsigned int>::iterator selection = _featureSelectionList.begin();
    unsigned int i = 0;
    for(auto feature : _featureList)
    {
        Eigen::VectorXd tmp;
        visp2eigen<Eigen::VectorXd>(feature->error(*(*desired_feature), *selection), tmp);
        _features_error.segment(i, tmp.size()) << -tmp;
        i += tmp.size();
        desired_feature++;
        selection++;
    }
    _b = _lambda*_features_error;
}

bool VisualServoing::setDesiredFeatures(std::list<vpBasicFeature *>& desired_feature_list)
{
    if(desired_feature_list.size() != _desiredFeatureList.size())
    {
        XBot::Logger::error("desired_feature_list.size() is %i, should be %i", desired_feature_list.size(), _desiredFeatureList.size());
        return false;
    }

    _desiredFeatureList = desired_feature_list;
    return true;
}               

bool VisualServoing::setFeatureSelectionList(std::list<unsigned int>& feature_selection_list)
{
    if(feature_selection_list.size() != _featureSelectionList.size())
    {
        XBot::Logger::error("feature_selection_list.size() is %i, should be %i", feature_selection_list.size(), _featureSelectionList.size());
        return false;
    }

    _featureSelectionList = feature_selection_list;
    return true;
}

const std::list<unsigned int>& VisualServoing::getFeatureListSelection() const
{
    return _featureSelectionList;
}

void VisualServoing::_log(XBot::MatLogger::Ptr logger)
{
    if(_L.rows() > 0)
        logger->add(_task_id + "_L", _L);

    if(_featureList.size() > 0)
    {
        int j = 0;
        std::list<vpBasicFeature *>::iterator feature;
        std::list<vpBasicFeature *>::iterator desired_feature;
        for(feature = _featureList.begin(), desired_feature = _desiredFeatureList.begin();
            feature != _featureList.end() && desired_feature != _desiredFeatureList.end();
            feature++, desired_feature++)
        {
            vpColVector s = (*feature)->get_s();
            vpColVector sd = (*desired_feature)->get_s();

            Eigen::VectorXd f(s.size()), fd(sd.size());
            for(unsigned int i = 0; i < s.size(); ++i)
            {
                f[i] = s[i];
                fd[i] = sd[i];
            }
            logger->add(_task_id +"_s"+std::to_string(j), f);
            logger->add(_task_id +"_sd"+std::to_string(j), fd);
            j++;
        }
    }

    _cartesian_task->log(logger);

}

const Eigen::VectorXd& VisualServoing::getFeaturesError() const
{
    return _features_error;
}

//VisualServoing::Ptr OpenSoT::tasks::velocity::operator%(const VisualServoing::Ptr task, const std::list<unsigned int>& rowIndices)
//{
//    task->_col_indices = rowIndices;
//    task->computeInteractionMatrix();
//    return task;
//}




