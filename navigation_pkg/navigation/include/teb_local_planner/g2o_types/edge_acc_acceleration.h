/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef EDGE_ACCACCELERATION_H_
#define EDGE_ACCACCELERATION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/Twist.h>



namespace navigation
{

/**
 * @class EdgeAccAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationStart
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() and EdgeAccelerationGoal() for defining boundary values!
 */    
class EdgeAccAcceleration : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */ 
  EdgeAccAcceleration()
  {
    this->resize(7);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]); 
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
    const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

    // VELOCITY & ACCELERATION
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    const Eigen::Vector2d diff3 = pose4->position() - pose3->position();
        
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    double dist3 = diff3.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    const double angle_diff3 = g2o::normalize_theta(pose4->theta() - pose3->theta());
    
    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }
        if (angle_diff3 != 0)
        {
            const double radius =  dist3/(2*sin(angle_diff3/2));
            dist3 = fabs( angle_diff3 * radius ); // actual arg length!
        }
    }
    
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    double vel3 = dist3 / dt3->dt();
    
    
    // consider directions
//     vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
//     vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta())); 
    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) ); 
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) ); 
    vel3 *= fast_sigmoid( 100*(diff3.x()*cos(pose3->theta()) + diff3.y()*sin(pose3->theta())) ); 
    
    const double acc_lin1  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );
    const double acc_lin2  = (vel3 - vel2)*2 / ( dt2->dt() + dt3->dt() );

    const double acc_acc_lin = (acc_lin2 - acc_lin1)/(0.25*dt1->dt() + 0.5*dt2->dt() + 0.25*dt3->dt());
   

    _error[0] = penaltyBoundToInterval(acc_acc_lin,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double omega3 = angle_diff3 / dt3->dt();

    const double acc_rot1  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
    const double acc_rot2  = (omega3 - omega2)*2 / ( dt2->dt() + dt3->dt() );

    const double acc_acc_rot =  (acc_rot2 - acc_rot1)/(0.25*dt1->dt() + 0.5*dt2->dt() + 0.25*dt3->dt());
      
    _error[1] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
  }



#ifdef USE_ANALYTIC_JACOBI
#if 0
  /*
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPointXY* conf1 = static_cast<const VertexPointXY*>(_vertices[0]);
    const VertexPointXY* conf2 = static_cast<const VertexPointXY*>(_vertices[1]);
    const VertexPointXY* conf3 = static_cast<const VertexPointXY*>(_vertices[2]);
    const VertexTimeDiff* deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexOrientation* angle1 = static_cast<const VertexOrientation*>(_vertices[5]);
    const VertexOrientation* angle2 = static_cast<const VertexOrientation*>(_vertices[6]);
    const VertexOrientation* angle3 = static_cast<const VertexOrientation*>(_vertices[7]);

    Eigen::Vector2d deltaS1 = conf2->estimate() - conf1->estimate();
    Eigen::Vector2d deltaS2 = conf3->estimate() - conf2->estimate();
    double dist1 = deltaS1.norm();
    double dist2 = deltaS2.norm();
    
    double sum_time = deltaT1->estimate() + deltaT2->estimate();
    double sum_time_inv = 1 / sum_time;
    double dt1_inv = 1/deltaT1->estimate();
    double dt2_inv = 1/deltaT2->estimate();
    double aux0 = 2/sum_time_inv;
    double aux1 = dist1 * deltaT1->estimate();
    double aux2 = dist2 * deltaT2->estimate();

    double vel1 = dist1 * dt1_inv;
    double vel2 = dist2 * dt2_inv;
    double omega1 = g2o::normalize_theta( angle2->estimate() - angle1->estimate() ) * dt1_inv;
    double omega2 = g2o::normalize_theta( angle3->estimate() - angle2->estimate() ) * dt2_inv;
    double acc = (vel2 - vel1) * aux0;
    double omegadot = (omega2 - omega1) * aux0;
    double aux3 = -acc/2;
    double aux4 = -omegadot/2;
    
    double dev_border_acc = penaltyBoundToIntervalDerivative(acc, tebConfig.robot_acceleration_max_trans,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    double dev_border_omegadot = penaltyBoundToIntervalDerivative(omegadot, tebConfig.robot_acceleration_max_rot,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    
    _jacobianOplus[0].resize(2,2); // conf1
    _jacobianOplus[1].resize(2,2); // conf2
    _jacobianOplus[2].resize(2,2); // conf3
    _jacobianOplus[3].resize(2,1); // deltaT1
    _jacobianOplus[4].resize(2,1); // deltaT2
    _jacobianOplus[5].resize(2,1); // angle1
    _jacobianOplus[6].resize(2,1); // angle2
    _jacobianOplus[7].resize(2,1); // angle3
    
    if (aux1==0) aux1=1e-20;
    if (aux2==0) aux2=1e-20;
  
    if (dev_border_acc!=0)
    {
      // TODO: double aux = aux0 * dev_border_acc;
      // double aux123 = aux / aux1;
      _jacobianOplus[0](0,0) = aux0 * deltaS1[0] / aux1 * dev_border_acc; // acc x1
      _jacobianOplus[0](0,1) = aux0 * deltaS1[1] / aux1 * dev_border_acc; // acc y1
      _jacobianOplus[1](0,0) = -aux0 * ( deltaS1[0] / aux1 + deltaS2[0] / aux2 ) * dev_border_acc; // acc x2
      _jacobianOplus[1](0,1) = -aux0 * ( deltaS1[1] / aux1 + deltaS2[1] / aux2 ) * dev_border_acc; // acc y2
      _jacobianOplus[2](0,0) = aux0 * deltaS2[0] / aux2 * dev_border_acc; // acc x3
      _jacobianOplus[2](0,1) = aux0 * deltaS2[1] / aux2 * dev_border_acc; // acc y3	
      _jacobianOplus[2](0,0) = 0;
      _jacobianOplus[2](0,1) = 0;
      _jacobianOplus[3](0,0) = aux0 * (aux3 + vel1 * dt1_inv) * dev_border_acc; // acc deltaT1
      _jacobianOplus[4](0,0) = aux0 * (aux3 - vel2 * dt2_inv) * dev_border_acc; // acc deltaT2
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // acc x1
      _jacobianOplus[0](0,1) = 0; // acc y1	
      _jacobianOplus[1](0,0) = 0; // acc x2
      _jacobianOplus[1](0,1) = 0; // acc y2
      _jacobianOplus[2](0,0) = 0; // acc x3
      _jacobianOplus[2](0,1) = 0; // acc y3	
      _jacobianOplus[3](0,0) = 0; // acc deltaT1
      _jacobianOplus[4](0,0) = 0; // acc deltaT2
    }
    
    if (dev_border_omegadot!=0)
    {
      _jacobianOplus[3](1,0) = aux0 * ( aux4 + omega1 * dt1_inv ) * dev_border_omegadot; // omegadot deltaT1
      _jacobianOplus[4](1,0) = aux0 * ( aux4 - omega2 * dt2_inv ) * dev_border_omegadot; // omegadot deltaT2
      _jacobianOplus[5](1,0) = aux0 * dt1_inv * dev_border_omegadot; // omegadot angle1
      _jacobianOplus[6](1,0) = -aux0 * ( dt1_inv + dt2_inv ) * dev_border_omegadot; // omegadot angle2
      _jacobianOplus[7](1,0) = aux0 * dt2_inv * dev_border_omegadot; // omegadot angle3
    }
    else
    {
      _jacobianOplus[3](1,0) = 0; // omegadot deltaT1
      _jacobianOplus[4](1,0) = 0; // omegadot deltaT2
      _jacobianOplus[5](1,0) = 0; // omegadot angle1
      _jacobianOplus[6](1,0) = 0; // omegadot angle2
      _jacobianOplus[7](1,0) = 0; // omegadot angle3			
    }

    _jacobianOplus[0](1,0) = 0; // omegadot x1
    _jacobianOplus[0](1,1) = 0; // omegadot y1
    _jacobianOplus[1](1,0) = 0; // omegadot x2
    _jacobianOplus[1](1,1) = 0; // omegadot y2
    _jacobianOplus[2](1,0) = 0; // omegadot x3
    _jacobianOplus[2](1,1) = 0; // omegadot y3
    _jacobianOplus[5](0,0) = 0; // acc angle1
    _jacobianOplus[6](0,0) = 0; // acc angle2
    _jacobianOplus[7](0,0) = 0; // acc angle3
    }
#endif
#endif

      
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};
    
    
/**
 * @class EdgeAccAccelerationStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
 */      
class EdgeAccAccelerationStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */	  
  EdgeAccAccelerationStart()
  {
    _measurement = NULL;
    this->resize(5);
  }
  
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff1 != 0)
    {
        const double radius =  dist1/(2*sin(angle_diff1/2));
        dist1 = fabs( angle_diff1 * radius ); // actual arg length!
    }
    if (cfg_->trajectory.exact_arc_length && angle_diff2 != 0)
    {
        const double radius =  dist2/(2*sin(angle_diff2/2));
        dist2 = fabs( angle_diff2 * radius ); // actual arg length!
    }
    
    const double vel1 = _measurement->linear.x;
    double vel2 = dist1 / dt1->dt();
    double vel3 = dist2 / dt2->dt();

    // consider directions
    //vel2 *= g2o::sign(diff[0]*cos(pose1->theta()) + diff[1]*sin(pose1->theta())); 
    vel2 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) ); 
    vel3 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) ); 

    const double acc_lin1  = (vel2 - vel1) / dt1->dt();
    const double acc_lin2  = (vel3 - vel2) / dt2->dt();

    const double acc_acc_lin = (acc_lin2 - acc_lin1)/(0.75*dt1->dt() + 0.25*dt2->dt());
   
    
    _error[0] = penaltyBoundToInterval(acc_acc_lin,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    const double omega1 = _measurement->angular.z;
    const double omega2 = angle_diff1 / dt1->dt();
    const double omega3 = angle_diff2 / dt2->dt();

    const double acc_rot1  = (omega2 - omega1) / dt1->dt();
    const double acc_rot2  = (omega3 - omega2) / dt2->dt();

    const double acc_acc_rot = (acc_rot2 - acc_rot1)/(0.75*dt1->dt() + 0.25*dt2->dt());
      
    _error[1] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
  }
  
  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
  
public:       
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};    
    
    
    

/**
 * @class EdgeAccAccelerationGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() for defining boundary (initial) values at the end of the trajectory
 */  
class EdgeAccAccelerationGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */  
  EdgeAccAccelerationGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }
  

  /**
   * @brief Actual cost function
   */ 
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION

    const Eigen::Vector2d diff1 = pose_pre_goal->position() - pose_pre_pre_goal->position();  
    const Eigen::Vector2d diff2 = pose_goal->position() - pose_pre_goal->position();  
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose_pre_goal->theta() - pose_pre_pre_goal->theta());
    const double angle_diff2 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
    if (cfg_->trajectory.exact_arc_length  && angle_diff1 != 0)
    {
        double radius =  dist1/(2*sin(angle_diff1/2));
        dist1 = fabs( angle_diff1 * radius ); // actual arg length!
    }
    if (cfg_->trajectory.exact_arc_length  && angle_diff2 != 0)
    {
        double radius =  dist2/(2*sin(angle_diff2/2));
        dist2 = fabs( angle_diff2 * radius ); // actual arg length!
    }
    
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    const double vel3 = _measurement->linear.x;
    
    // consider directions
    //vel1 *= g2o::sign(diff[0]*cos(pose_pre_goal->theta()) + diff[1]*sin(pose_pre_goal->theta())); 
    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose_pre_pre_goal->theta()) + diff1.y()*sin(pose_pre_pre_goal->theta())) ); 
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose_pre_goal->theta()) + diff2.y()*sin(pose_pre_goal->theta())) ); 
    
    const double acc_lin1  = (vel2 - vel1) / dt1->dt();
    const double acc_lin2  = (vel3 - vel2) / dt2->dt();

    const double acc_acc_lin = (acc_lin2 - acc_lin1)/(0.25*dt1->dt() + 0.75*dt2->dt());

    _error[0] = penaltyBoundToInterval(acc_acc_lin,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double omega3 = _measurement->angular.z;

    const double acc_rot1  = (omega2 - omega1) / dt1->dt();
    const double acc_rot2  = (omega3 - omega2) / dt2->dt();

    const double acc_acc_rot = (acc_rot2 - acc_rot1)/(0.25*dt1->dt() + 0.75*dt2->dt());
      
    _error[1] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
  }
    
  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; 
    



/**
 * @class EdgeAccAccelerationHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot } ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x position parts of all three poses \n
 * \e ay is calculated using the difference quotient (twice) and the y position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration (x-dir), 
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomicStart
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() and EdgeAccelerationHolonomicGoal() for defining boundary values!
 */    
class EdgeAccAccelerationHolonomic : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */    
  EdgeAccAccelerationHolonomic()
  {
    this->resize(7);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
    const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    Eigen::Vector2d diff3 = pose4->position() - pose3->position();
    
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta()); 
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta()); 
    double cos_theta3 = std::cos(pose3->theta());
    double sin_theta3 = std::sin(pose3->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
    double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
    double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();
    // transform pose4 into robot frame pose3 (inverse 2d rotation matrix)
    double p3_dx =  cos_theta3*diff3.x() + sin_theta3*diff3.y();
    double p3_dy = -sin_theta3*diff3.x() + cos_theta3*diff3.y();
    
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();
    double vel3_x = p3_dx / dt3->dt();
    double vel3_y = p3_dy / dt3->dt();
    
    double dt12 = dt1->dt() + dt2->dt();
    double dt23 = dt2->dt() + dt3->dt();
    
    double acc_x1  = (vel2_x - vel1_x)*2 / dt12;
    double acc_y1  = (vel2_y - vel1_y)*2 / dt12;
    double acc_x2  = (vel3_x - vel2_x)*2 / dt23;
    double acc_y2  = (vel3_y - vel2_y)*2 / dt23;

    double acc_acc_x = (acc_x2 - acc_x1)/(0.25*dt1->dt() + 0.5*dt2->dt() + 0.25*dt3->dt()); 
    double acc_acc_y = (acc_y2 - acc_y1)/(0.25*dt1->dt() + 0.5*dt2->dt() + 0.25*dt3->dt()); 
   
    _error[0] = penaltyBoundToInterval(acc_acc_x,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_acc_y,cfg_->robot.acc_acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
    double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
    double omega3 = g2o::normalize_theta(pose4->theta() - pose3->theta()) / dt3->dt();

    double acc_rot1  = (omega2 - omega1)*2 / dt12;
    double acc_rot2  = (omega3 - omega2)*2 / dt23;

    double acc_acc_rot = (acc_rot2 - acc_rot1)/(0.25*dt1->dt() + 0.5*dt2->dt() + 0.25*dt3->dt());
      
    _error[2] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccAcceleration::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccAcceleration::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};


/**
 * @class EdgeAccelerationHolonomicStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses. \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicGoal() for defining boundary values at the end of the trajectory!
 */      
class EdgeAccAccelerationHolonomicStart : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */   
  EdgeAccAccelerationHolonomicStart()
  {
    this->resize(5);
    _measurement = NULL;
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    Eigen::Vector2d diff2 = pose3->position() - pose2->position();
            
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta()); 
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
    double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
    double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
    double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();
    
    double vel1_x = _measurement->linear.x;
    double vel1_y = _measurement->linear.y;
    double vel2_x = p1_dx / dt1->dt();
    double vel2_y = p1_dy / dt1->dt();
    double vel3_x = p2_dx / dt2->dt();
    double vel3_y = p2_dy / dt2->dt();

    double acc_lin_x1  = (vel2_x - vel1_x) / dt1->dt();
    double acc_lin_y1  = (vel2_y - vel1_y) / dt1->dt();
    double acc_lin_x2  = (vel3_x - vel2_x) / dt2->dt();
    double acc_lin_y2  = (vel3_y - vel2_y) / dt2->dt();

    double acc_acc_lin_x  = (acc_lin_x2 - acc_lin_x1) / (0.75*dt1->dt() + 0.25*dt2->dt()); 
    double acc_acc_lin_y  = (acc_lin_y2 - acc_lin_y1) / (0.75*dt1->dt() + 0.25*dt2->dt());
    
    _error[0] = penaltyBoundToInterval(acc_acc_lin_x,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_acc_lin_y,cfg_->robot.acc_acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = _measurement->angular.z;
    double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
    double omega3 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();

    double acc_rot1  = (omega2 - omega1) / dt1->dt();
    double acc_rot2  = (omega3 - omega2) / dt2->dt();

    double acc_acc_rot = (acc_rot2 - acc_rot1)/(0.75*dt1->dt() + 0.25*dt2->dt());
      
    _error[2] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccAccelerationStart::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccAccelerationStart::computeError() rotational: _error[2]=%f\n",_error[2]);
  }
  
  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
        
public:       
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};    
    
       

/**
 * @class EdgeAccAccelerationHolonomicGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one is the strafing velocity and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() for defining boundary (initial) values at the end of the trajectory
 */  
class EdgeAccAccelerationHolonomicGoal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */  
  EdgeAccAccelerationHolonomicGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }
  
  /**
   * @brief Actual cost function
   */ 
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION

    Eigen::Vector2d diff1 = pose_pre_goal->position() - pose_pre_pre_goal->position();    
    Eigen::Vector2d diff2 = pose_goal->position() - pose_pre_goal->position();    
    
    double cos_theta1 = std::cos(pose_pre_pre_goal->theta());
    double sin_theta1 = std::sin(pose_pre_pre_goal->theta()); 
    double cos_theta2 = std::cos(pose_pre_goal->theta());
    double sin_theta2 = std::sin(pose_pre_goal->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
    double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
    double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
    double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();
   
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();
    double vel3_x = _measurement->linear.x;
    double vel3_y = _measurement->linear.y;
    
    double acc_lin_x1  = (vel2_x - vel1_x) / dt1->dt();
    double acc_lin_y1  = (vel2_y - vel1_y) / dt1->dt();
    double acc_lin_x2  = (vel3_x - vel2_x) / dt2->dt();
    double acc_lin_y2  = (vel3_y - vel2_y) / dt2->dt();

    double acc_acc_lin_x  = (acc_lin_x2 - acc_lin_x1) / (0.25*dt1->dt() + 0.75*dt2->dt()); 
    double acc_acc_lin_y  = (acc_lin_y2 - acc_lin_y1) / (0.25*dt1->dt() + 0.75*dt2->dt());

    _error[0] = penaltyBoundToInterval(acc_acc_lin_x,cfg_->robot.acc_acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_acc_lin_y,cfg_->robot.acc_acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose_pre_goal->theta() - pose_pre_pre_goal->theta()) / dt1->dt();
    double omega2 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt2->dt();
    double omega3 = _measurement->angular.z;

    double acc_rot1  = (omega2 - omega1) / dt1->dt();
    double acc_rot2  = (omega3 - omega2) / dt2->dt();

    double acc_acc_rot = (acc_rot2 - acc_rot1)/(0.25*dt1->dt() + 0.75*dt2->dt());
      
    _error[2] = penaltyBoundToInterval(acc_acc_rot,cfg_->robot.acc_acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccAccelerationGoal::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccAccelerationGoal::computeError() rotational: _error[2]=%f\n",_error[2]);
  }
  
  
  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }
  

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; 
    



}; // end namespace

#endif /* EDGE_ACCELERATION_H_ */
