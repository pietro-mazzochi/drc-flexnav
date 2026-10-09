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

#ifndef STATE_FEEDBACK_LINEARIZATION_H
#define STATE_FEEDBACK_LINEARIZATION_H

#include <cstdio>
#include <iostream>
#include "Eigen/Dense"

enum
{
  MODEL_PARAMETER_K5_INDEX = 0,
  MODEL_PARAMETER_K6_INDEX,
  MODEL_PARAMETER_K7_INDEX,
  MODEL_PARAMETER_K8_INDEX,
  MODEL_PARAMETER_KV_INDEX,
  MODEL_PARAMETER_KW_INDEX,
  MODEL_PARAMETER_INDEX_SIZE
};

typedef Eigen::Matrix<double, MODEL_PARAMETER_INDEX_SIZE, 1> ModelParametersVector;

class StateFeedbackLinearization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateFeedbackLinearization(const ModelParametersVector params);
  StateFeedbackLinearization(const double K5, const double K6, const double K7, const double K8, const double Kv, const double Kw);
  ~StateFeedbackLinearization(void);

  void get_model_parameters(double &K5, double &K6, double &K7, double &K8, double &Kv, double &Kw);
  void set_model_parameters(const double K5, const double K6, const double K7, const double K8, const double Kv, const double Kw);

  ModelParametersVector get_model_parameters_vector(void);
  void set_model_parameters_vector(const ModelParametersVector params);

  Eigen::MatrixXd get_F_matrix(void);
  Eigen::MatrixXd get_G_matrix(void);
  Eigen::MatrixXd get_inverse_G_matrix(void);

  Eigen::Vector2d compute_system_inputs(const Eigen::Vector2d nu, const Eigen::Vector2d u);

private:
  void set_F_matrix(void);
  void set_G_matrix(void);
  Eigen::Vector2d nu_;
  Eigen::Vector2d tau_;
  Eigen::Matrix<double, 2, 4> Fu_;
  Eigen::Matrix<double, 2, 2> G_;
  Eigen::Matrix<double, 2, 2> Ginv_;
  ModelParametersVector params_;
};

#endif
