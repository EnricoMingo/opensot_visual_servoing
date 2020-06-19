#include <opensot_visual_seroving/tasks/velocity/VisualServoing.h>
#include <visp/vpServo.h>


using namespace OpenSoT::tasks::velocity;

VisualServoing::VisualServoing(std::string task_id,
                               const Eigen::VectorXd &x,
                               XBot::ModelInterface &robot,
                               std::string base_link,
                               std::string camera_link):
    Cartesian(task_id, x, robot, camera_link, base_link)
{
    _lambda = 0.0; //visual servoing task does not use feedback control law in Cartesian task

    _V.setIdentity(); //intialize sensor_frame and camera_frame coincident

    _is_body_jacobian = true; //we want to control the robot in camera_frame
}

void VisualServoing::_update(const Eigen::VectorXd &x)
{
    if (!_is_body_jacobian)
    {
        std::cerr << "_is_body_jacobian has to be set to true! Setting _is_body_jacobian to true" << std::endl;
        _is_body_jacobian = true;
    }

    //1) computes Cartesian quantities from Cartesian task
    Cartesian::_update(x);

    _J = _A;
    _v = _b;

    //2) computes new Jacobian and _b term using the interaction matrix from VISP
    _J = _V*_A;
    _A = _L*_J;
    _b = _v;
}

void VisualServoing::setVelocityTwistMatrix(const Eigen::Matrix6d& V)
{
    _V = V;
}

void VisualServoing::getVelocityTwistMatrix(Eigen::Matrix6d& V)
{
    V = _V;
}

void VisualServoing::computeInteractionMatrix()
{
    computeInteractionMatrixFromList(_featureList, _featureSelectionList, _L_visp);
    visp2eigen<Eigen::MatrixXd>(_L_visp, _L);
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
