/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

// Author(s): Matei Ciocarlie

#ifndef _DATABASE_GRASP_H_
#define _DATABASE_GRASP_H_

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <database_interface/db_class.h>

#include <boost/shared_ptr.hpp>

namespace household_objects_database {

/* Database convention is currently as follows:

   - joints are represented in an array with the value of each DOF of the robot: {j1} 
     (only one joint for the PR2)

   - position and orientation are also in an array, with pose as a translation and quaternion: 
     {tx, ty, tz, qw, qx, qy, qz}
     WARNING: in the database, the w component of the quaternion is first!!!

   WARNING: database units are mm, ROS units are m.
     
   For now we use the stored "energy" from the database as grasp quality; this is OK, though
   in the future we hope to have better measures. For now, this has the counterintuitive effect
   that lower values are better, which is what the term "energy" implies. However, in this
   class we use it as "quality" which usually means that higher is better.
*/


//! A wrapper for the geometry_msgs::pose so we can store it in the database
/*! Right now, in the database, the pose is stored in a weird way (see above), so we need a
  wrapper to take care of the conversion. In the future, hopefully we'll get rid of this.
 */
class DatabaseHandPose
{
 public:
  geometry_msgs::Pose pose_;
};

std::istream& operator >> (std::istream &str, DatabaseHandPose &dhp);
std::ostream& operator << (std::ostream &str, const DatabaseHandPose &dhp);

//! A wrapper for a vector of doubles, used in the database to store the hand joint angles
/*! Right now, in the database, the posture is stored in a weird way (see above), so we need a
  wrapper to take care of the conversion. In the future, hopefully we'll get rid of this.
 */
class DatabaseHandPosture
{
 public:
  std::vector<double> joint_angles_;
};

std::istream& operator >> (std::istream &str, DatabaseHandPosture &dhp);
std::ostream& operator << (std::ostream &str, const DatabaseHandPosture &dhp);

//! Contains a database record of a grasp
class DatabaseGrasp : public database_interface::DBClass
{
 private:

 public:
  //! The primary key, id of this grasp in the database
  database_interface::DBField<int> id_;
  //! The scaled model that this grasp is for
  database_interface::DBField<int> scaled_model_id_;

  //! The pre-grasp pose
  database_interface::DBField<DatabaseHandPose> pre_grasp_pose_;
  //! The pre-grasp posture
  database_interface::DBField<DatabaseHandPosture> pre_grasp_posture_;

  //! The final grasp pose
  database_interface::DBField<DatabaseHandPose> final_grasp_pose_;
  //! The final grasp posture
  database_interface::DBField<DatabaseHandPosture> final_grasp_posture_;

  //! The quality of the grasp (lower means better)
  database_interface::DBField<double> quality_;
  //! The pre-grasp clearance (distance in mm between hand and object for pregrasp)
  database_interface::DBField<double> pre_grasp_clearance_;
  //! Whether this grasp is the representative for a cluster of grasps
  database_interface::DBField<bool> cluster_rep_;
  //! The clearance between the grasp and a table with hard-coded characteristics for this grasp
  database_interface::DBField<double> table_clearance_;

  //! The name of the hand that this grasp is for
  database_interface::DBField<std::string> hand_name_;

  //! Only initialized fields
  DatabaseGrasp() :
   id_(database_interface::DBFieldBase::TEXT, this, "grasp_id", "grasp", true),
   scaled_model_id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "grasp", true),
   pre_grasp_pose_(database_interface::DBFieldBase::TEXT, this, "grasp_pregrasp_position", "grasp", true),
   pre_grasp_posture_(database_interface::DBFieldBase::TEXT, this, "grasp_pregrasp_joints", "grasp", true),
   final_grasp_pose_(database_interface::DBFieldBase::TEXT, this, "grasp_grasp_position", "grasp", true),
   final_grasp_posture_(database_interface::DBFieldBase::TEXT, this, "grasp_grasp_joints", "grasp", true),
   quality_(database_interface::DBFieldBase::TEXT, this, "grasp_energy", "grasp", true),
   pre_grasp_clearance_(database_interface::DBFieldBase::TEXT, this, "grasp_pregrasp_clearance", "grasp", true),
   cluster_rep_(database_interface::DBFieldBase::TEXT, this, "grasp_cluster_rep", "grasp", true),
   table_clearance_(database_interface::DBFieldBase::TEXT, this, "grasp_table_clearance", "grasp", true),
   hand_name_(database_interface::DBFieldBase::TEXT, this, "hand_name", "grasp", true)
  {
    //primary key field
    primary_key_field_ = &id_;
    //all the other fields
    fields_.push_back(&scaled_model_id_);
    fields_.push_back(&pre_grasp_pose_);
    fields_.push_back(&pre_grasp_posture_);
    fields_.push_back(&final_grasp_pose_);
    fields_.push_back(&final_grasp_posture_);
    fields_.push_back(&quality_);
    fields_.push_back(&pre_grasp_clearance_);
    fields_.push_back(&cluster_rep_);
    fields_.push_back(&table_clearance_);
    fields_.push_back(&hand_name_);

    //sequences
    id_.setSequenceName("grasp_grasp_id_seq");

    //by default, all fields here are used, and many of then not-null, so sync both ways
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
    //primary key id_ only syncs from database; it has a sequence which is used by default on insertions
    id_.setWriteToDatabase(false);
  }

  //! Empty stub
  ~DatabaseGrasp(){}
};


typedef boost::shared_ptr<DatabaseGrasp> DatabaseGraspPtr;

} //namespace

#endif
