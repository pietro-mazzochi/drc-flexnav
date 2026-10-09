// DRC Flexnav Controllers
// State Feedback Linearization
//
// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "drc_flexnav_controllers/state_feedback_linearization.hpp"

StateFeedbackLinearization::StateFeedbackLinearization(const ModelParametersVector params)
{
  this->Fu_.setZero();
  this->G_.setZero();
  this->Ginv_.setZero();

  this->params_ = params;
  this->set_model_parameters_vector(params);

  this->set_F_matrix();
  this->set_G_matrix();
}

StateFeedbackLinearization::StateFeedbackLinearization(const double K5, const double K6, const double K7, const double K8, const double Kv, const double Kw)
{
  this->Fu_.setZero();
  this->G_.setZero();
  this->Ginv_.setZero();

  this->set_model_parameters(K5, K6, K7, K8, Kv, Kw);

  this->set_F_matrix();
  this->set_G_matrix();
}

StateFeedbackLinearization::~StateFeedbackLinearization()
{
}

Eigen::MatrixXd StateFeedbackLinearization::get_F_matrix(void)
{
  return this->Fu_;
}

Eigen::MatrixXd StateFeedbackLinearization::get_G_matrix(void)
{
  return this->G_;
}

Eigen::MatrixXd StateFeedbackLinearization::get_inverse_G_matrix(void)
{
  return this->Ginv_;
}

Eigen::Vector2d StateFeedbackLinearization::compute_system_inputs(const Eigen::Vector2d nu, const Eigen::Vector2d u)
{
  nu_ = nu;

  // uf = [v; w; vw; ww]'
  Eigen::Matrix<double, 4, 1> uf_;
  uf_ << u[0], u[1], u[0] * u[1], u[1] * u[1];

  // tau = Ginv*(dudt + F*u);
  this->tau_ = Ginv_ * (nu_ - (Fu_ * uf_));

  return this->tau_;
}

void StateFeedbackLinearization::set_F_matrix(void)
{
  // F = [ kv, 0, 0, k5; 0, kw, k6, 0]
  this->Fu_ << params_[MODEL_PARAMETER_KV_INDEX], 0, 0, params_[MODEL_PARAMETER_K5_INDEX],
      0, params_[MODEL_PARAMETER_KW_INDEX], params_[MODEL_PARAMETER_K6_INDEX], 0;

  std::cout << " Dynamics linearizing controller " << std::endl;

  std::cout << "   - F matrix: " << std::endl
            << Fu_ << std::endl;
}

void StateFeedbackLinearization::set_G_matrix(void)
{
  // G = [k7, k7; k8, -k8];
  this->G_ << params_[MODEL_PARAMETER_K7_INDEX], params_[MODEL_PARAMETER_K7_INDEX],
      params_[MODEL_PARAMETER_K8_INDEX], -params_[MODEL_PARAMETER_K8_INDEX];
  this->Ginv_ = this->G_.inverse();

  std::cout << " Dynamics linearizing controller " << std::endl;

  std::cout << "   - G matrix: " << std::endl
            << G_ << std::endl;
  std::cout << "   - Inverse of G matrix: " << std::endl
            << Ginv_ << std::endl;
}

ModelParametersVector StateFeedbackLinearization::get_model_parameters_vector(void)
{
  return this->params_;
}

void StateFeedbackLinearization::set_model_parameters_vector(const ModelParametersVector params)
{
  this->params_ = params;

  std::cout << " Dynamics linearizing controller " << std::endl;

  std::cout << "   - K5: " << this->params_[MODEL_PARAMETER_K5_INDEX] << std::endl;
  std::cout << "   - K6: " << this->params_[MODEL_PARAMETER_K6_INDEX] << std::endl;
  std::cout << "   - K7: " << this->params_[MODEL_PARAMETER_K7_INDEX] << std::endl;
  std::cout << "   - K8: " << this->params_[MODEL_PARAMETER_K8_INDEX] << std::endl;
  std::cout << "   - KV: " << this->params_[MODEL_PARAMETER_KV_INDEX] << std::endl;
  std::cout << "   - KW: " << this->params_[MODEL_PARAMETER_KW_INDEX] << std::endl;
}

void StateFeedbackLinearization::get_model_parameters(double &K5, double &K6, double &K7, double &K8, double &Kv, double &Kw)
{
  K5 = this->params_[MODEL_PARAMETER_K5_INDEX];
  K6 = this->params_[MODEL_PARAMETER_K6_INDEX];
  K7 = this->params_[MODEL_PARAMETER_K7_INDEX];
  K8 = this->params_[MODEL_PARAMETER_K8_INDEX];
  Kv = this->params_[MODEL_PARAMETER_KV_INDEX];
  Kw = this->params_[MODEL_PARAMETER_KW_INDEX];
}

void StateFeedbackLinearization::set_model_parameters(const double K5, const double K6, const double K7, const double K8, const double Kv, const double Kw)
{
  this->params_[MODEL_PARAMETER_K5_INDEX] = K5;
  this->params_[MODEL_PARAMETER_K6_INDEX] = K6;
  this->params_[MODEL_PARAMETER_K7_INDEX] = K7;
  this->params_[MODEL_PARAMETER_K8_INDEX] = K8;
  this->params_[MODEL_PARAMETER_KV_INDEX] = Kv;
  this->params_[MODEL_PARAMETER_KW_INDEX] = Kw;

  std::cout << " Dynamics linearizing controller " << std::endl;

  std::cout << "   - K5: " << this->params_[MODEL_PARAMETER_K5_INDEX] << std::endl;
  std::cout << "   - K6: " << this->params_[MODEL_PARAMETER_K6_INDEX] << std::endl;
  std::cout << "   - K7: " << this->params_[MODEL_PARAMETER_K7_INDEX] << std::endl;
  std::cout << "   - K8: " << this->params_[MODEL_PARAMETER_K8_INDEX] << std::endl;
  std::cout << "   - KV: " << this->params_[MODEL_PARAMETER_KV_INDEX] << std::endl;
  std::cout << "   - KW: " << this->params_[MODEL_PARAMETER_KW_INDEX] << std::endl;
}
