#include "floatingBaseModel.hpp"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <stdio.h>

// #define THIS_COM "/home/action/程序/Actiger/controllers/Actiger_controller/controller/model/urdf/"
#define THIS_COM "/home/chen/Actiger/controllers/Actiger_controller/controller/model/urdf/"


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace Eigen;
using namespace quadruped;

FloatingBaseModel::FloatingBaseModel(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (
              THIS_COM"actiger_model.urdf", model_, true, false)) {
    std::cerr << "Error loading model actiger_model.urdf" << std::endl;
    abort();
  }

  dyn_model_ = new DynModel(model_);
  kin_model_ = new KinModel(model_);

  printf("[Actiger Model] Contructed\n");
}

FloatingBaseModel::~FloatingBaseModel(){
  delete dyn_model_;
  delete kin_model_;
  delete model_;
}

void FloatingBaseModel::PrintLinkList() {
    kin_model_->displaylinks();
}
unsigned int FloatingBaseModel:: FindLinkId(const char* _link_name) {
    kin_model_->find_body_id(_link_name);

    return 0;
}

void FloatingBaseModel::UpdateSystem(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
  dynacore::Vector qddot = qdot; qddot.setZero();
  UpdateKinematicsCustom(*model_, &q, &qdot, &qddot);
  dyn_model_->UpdateDynamics(q, qdot);
  kin_model_->UpdateKinematics(q, qdot);
}

void FloatingBaseModel::getCentroidInertia(dynacore::Matrix & Icent) const {
  kin_model_->getCentroidInertia(Icent);
}

void FloatingBaseModel::getCentroidJacobian(dynacore::Matrix & Jcent) const {
  Jcent.setZero();
  kin_model_->getCentroidJacobian(Jcent);
}

bool FloatingBaseModel::getInverseMassInertia(dynacore::Matrix & Ainv) const {
  return dyn_model_->getInverseMassInertia(Ainv);
}

bool FloatingBaseModel::getMassInertia(dynacore::Matrix & A) const {
  return dyn_model_->getMassInertia(A);
}

bool FloatingBaseModel::getGravity(dynacore::Vector & grav) const {
  return dyn_model_->getGravity(grav);
}

bool FloatingBaseModel::getCoriolis(dynacore::Vector & coriolis) const {
  return dyn_model_->getCoriolis(coriolis);
}

void FloatingBaseModel::getFullJacobian(int link_id, dynacore::Matrix & J) const {
  //J = dynacore::Matrix::Zero(6, mercury::num_qdot);
  kin_model_->getJacobian(link_id, J);
}

void FloatingBaseModel::getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const{
    kin_model_->getJDotQdot(link_id, JDotQdot);
}

void FloatingBaseModel::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void FloatingBaseModel::getOri(int link_id, dynacore::Vect3 & rpy) const {
    kin_model_->getOri(link_id, rpy);
}
void FloatingBaseModel::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}
void FloatingBaseModel::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void FloatingBaseModel::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void FloatingBaseModel::getCoMPosition(dynacore::Vect3 & com_pos) const {
  com_pos = kin_model_->com_pos_;
}

void FloatingBaseModel::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}
void FloatingBaseModel::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
  centroid_vel = kin_model_->centroid_vel_;
}
void FloatingBaseModel::getWorld2BodyMatrix(dynacore::Mat3 & _World2Body){
  kin_model_ -> getWorldToBodyMatrix(_World2Body);
}