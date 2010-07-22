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

//! Wraps around the most common functionality of the objects database and offers it
//! as ROS services

#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <object_manipulation_msgs/GraspPlanning.h>

#include <household_objects_database_msgs/GetModelList.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include "household_objects_database/objects_database.h"

const std::string GET_MODELS_SERVICE_NAME = "get_model_list";
const std::string GET_MESH_SERVICE_NAME = "get_model_mesh";
const std::string GRASP_PLANNING_SERVICE_NAME = "database_grasp_planning";

using namespace household_objects_database_msgs;
using namespace household_objects_database;
using namespace object_manipulation_msgs;

//! Retrieves hand description info from the parameter server
/*! Duplicated from object_manipulator to avoid an additional dependency */
class HandDescription
{
 private:
  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

  inline std::string getStringParam(std::string name)
  {
    std::string value;
    if (!root_nh_.getParamCached(name, value))
    {
      ROS_ERROR("Hand description: could not find parameter %s", name.c_str());
    }
    //ROS_INFO_STREAM("Hand description param " << name << " resolved to " << value);
    return value;
  }

  inline std::vector<std::string> getVectorParam(std::string name)
  {
    std::vector<std::string> values;
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list)) 
    {
      ROS_ERROR("Hand description: could not find parameter %s", name.c_str());
    }
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Hand description: bad parameter %s", name.c_str());
    }
    //ROS_INFO_STREAM("Hand description vector param " << name << " resolved to:");
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
	ROS_ERROR("Hand description: bad parameter %s", name.c_str());
      }
      values.push_back( static_cast<std::string>(list[i]) );
      //ROS_INFO_STREAM("  " << values.back());
    }
    return values;	
  }

 public:
 HandDescription() : root_nh_("~") {}
  
  inline std::string handDatabaseName(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/hand_database_name");
  }
  
  inline std::vector<std::string> handJointNames(std::string arm_name)
  {
    return getVectorParam("/hand_description/" + arm_name + "/hand_joints");
  }
  
};

//! Wraps around database connection to provide database-related services through ROS
/*! Contains very thin wrappers for getting a list of scaled models and for getting the mesh
  of a model, as well as a complete server for the grasp planning service */
class ObjectsDatabaseNode
{
private:
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

  //! Server for the get models service
  ros::ServiceServer get_models_srv_;

  //! Server for the get mesh service
  ros::ServiceServer get_mesh_srv_;

  //! Server for the get grasps service
  ros::ServiceServer grasp_planning_srv_;

  //! The database connection itself
  ObjectsDatabase *database_;

  //! Callback for the get models service
  bool getModelsCB(GetModelList::Request &request, GetModelList::Response &response)
  {
    if (!database_)
    {
      response.return_code.code = response.return_code.DATABASE_NOT_CONNECTED;
      return true;
    }
    std::vector< boost::shared_ptr<DatabaseScaledModel> > models;
    if (!database_->getScaledModelsBySet(models, request.model_set))
    {
      response.return_code.code = response.return_code.DATABASE_QUERY_ERROR;
      return true;
    }
    for (size_t i=0; i<models.size(); i++)
    {
      response.model_ids.push_back( models[i]->id_.data() );
    }
    response.return_code.code = response.return_code.SUCCESS;
    return true;
  }

  //! Callback for the get mesh service
  bool getMeshCB(GetModelMesh::Request &request, GetModelMesh::Response &response)
  {
    if (!database_)
    {
      response.return_code.code = response.return_code.DATABASE_NOT_CONNECTED;
      return true;
    }
    if ( !database_->getScaledModelMesh(request.model_id, response.mesh) )
    {
      response.return_code.code = response.return_code.DATABASE_QUERY_ERROR;
      return true;
    }
    response.return_code.code = response.return_code.SUCCESS;
    return true;
  }


  //! Prune grasps that require gripper to be open all the way, or that are marked in db as colliding with table
  /*! Use negative value for table_clearance_threshold if no clearing should be done
    based on table clearance.
  */
  virtual void pruneGraspList(std::vector< boost::shared_ptr<DatabaseGrasp> > &grasps,
			      double gripper_threshold, 
			      double table_clearance_threshold)
  {
    std::vector< boost::shared_ptr<DatabaseGrasp> >::iterator prune_it = grasps.begin();
    int pruned = 0;
    while ( prune_it != grasps.end() )
    {
      //by mistake, table clearance in database is currently in mm
      if ((*prune_it)->final_grasp_posture_.get().joint_angles_[0] > gripper_threshold ||
	  (table_clearance_threshold >= 0.0 && (*prune_it)->table_clearance_.get() < table_clearance_threshold*1.0e3) ) 
      {
	prune_it = grasps.erase(prune_it);
	pruned++;
      } 
      else 
      {
	prune_it++;
      }
    }
    ROS_INFO("Database grasp planner: pruned %d grasps for table collision or gripper angle above threshold", pruned);
  }

  //! Callback for the get grasps service
  bool graspPlanningCB(GraspPlanning::Request &request, GraspPlanning::Response &response)
  {
    if (!database_)
    {
      ROS_ERROR("Database grasp planning: database not connected");
      return false;
    }

    HandDescription hd;
    int model_id = request.target.model_pose.model_id;
    std::string hand_id = hd.handDatabaseName(request.arm_name);
    
    //retrieve the raw grasps from the database
    std::vector< boost::shared_ptr<DatabaseGrasp> > grasps;
    if (!database_->getClusterRepGrasps(model_id, hand_id, grasps))
    {
      ROS_ERROR("Database grasp planning: database query error");
      return false;
    }
    ROS_INFO("Database object node: retrieved %u grasps from database", (unsigned int)grasps.size());
    
    //prune the retrieved grasps
    //pruning should be optional, based on node parameters
    pruneGraspList(grasps, 0.50, 0.0);

    //convert to the Grasp data type
    std::vector< boost::shared_ptr<DatabaseGrasp> >::iterator it;
    for (it = grasps.begin(); it != grasps.end(); it++)
    {
      ROS_ASSERT( (*it)->final_grasp_posture_.get().joint_angles_.size() == 
		  (*it)->pre_grasp_posture_.get().joint_angles_.size() );
      Grasp grasp;
      std::vector<std::string> joint_names = hd.handJointNames(request.arm_name);

      if (hand_id != "WILLOW_GRIPPER_2010")
      {
	//check that the number of joints in the ROS description of this hand
	//matches the number of values we have in the database
	if (joint_names.size() != (*it)->final_grasp_posture_.get().joint_angles_.size())
	{
	  ROS_ERROR("Database grasp specification does not match ROS description of hand. "
		    "Hand is expected to have %d joints, but database grasp specifies %d values", 
		    (int)joint_names.size(), (*it)->final_grasp_posture_.get().joint_angles_.size());
	  continue;
	}
	//for now we silently assume that the order of the joints in the ROS description of
	//the hand is the same as in the database description
	grasp.pre_grasp_posture.name = joint_names;
	grasp.grasp_posture.name = joint_names;
	grasp.pre_grasp_posture.position = (*it)->pre_grasp_posture_.get().joint_angles_;
	grasp.grasp_posture.position = (*it)->final_grasp_posture_.get().joint_angles_;	
      }
      else
      {
	//unfortunately we have to hack this, as the grasp is really defined by a single
	//DOF, but the urdf for the PR2 gripper is not well set up to do that
	if ( joint_names.size() != 4 || (*it)->final_grasp_posture_.get().joint_angles_.size() != 1)
	{
	  ROS_ERROR("PR2 gripper specs and database grasp specs do not match expected values");
	  continue;
	}
	grasp.pre_grasp_posture.name = joint_names;
	grasp.grasp_posture.name = joint_names;
	//replicate the single value from the database 4 times
	grasp.pre_grasp_posture.position.resize( joint_names.size(), 
						 (*it)->pre_grasp_posture_.get().joint_angles_.at(0));
	grasp.grasp_posture.position.resize( joint_names.size(), 
					     (*it)->final_grasp_posture_.get().joint_angles_.at(0));
      }
      //for now the effort is not in the database so we hard-code it in here
      //this will change at some point
      grasp.grasp_posture.effort.resize(joint_names.size(), 10000);
      //the pose of the grasp
      grasp.grasp_pose = (*it)->final_grasp_pose_.get().pose_;

      //insert the new grasp in the list
      response.planned_grasps.push_back(grasp);
    }

    ROS_INFO("Database grasp planner: returning %u grasps", (unsigned int)response.planned_grasps.size());
    return true;
  }

public:
  ObjectsDatabaseNode() : priv_nh_("~"), root_nh_("")
  {
    //initialize database connection
    std::string database_host, database_port, database_user, database_pass, database_name;
    root_nh_.param<std::string>("/model_database/database_host", database_host, "");
    int port_int;
    root_nh_.param<int>("/model_database/database_port", port_int, -1);
    std::stringstream ss; ss << port_int; database_port = ss.str();
    root_nh_.param<std::string>("/model_database/database_user", database_user, "");
    root_nh_.param<std::string>("/model_database/database_pass", database_pass, "");
    root_nh_.param<std::string>("/model_database/database_name", database_name, "");
    database_ = new ObjectsDatabase(database_host, database_port, database_user, database_pass, database_name);
    if (!database_->isConnected())
    {
      ROS_ERROR("ObjectsDatabaseNode: failed to open model database on host "
		"%s, port %s, user %s with password %s, database %s. Unable to do grasp "
		"planning on database recognized objects. Exiting.",
		database_host.c_str(), database_port.c_str(), 
		database_user.c_str(), database_pass.c_str(), database_name.c_str());
      delete database_; database_ = NULL;
    }

    //advertise services
    get_models_srv_ = priv_nh_.advertiseService(GET_MODELS_SERVICE_NAME, &ObjectsDatabaseNode::getModelsCB, this);    
    get_mesh_srv_ = priv_nh_.advertiseService(GET_MESH_SERVICE_NAME, &ObjectsDatabaseNode::getMeshCB, this);    
    grasp_planning_srv_ = priv_nh_.advertiseService(GRASP_PLANNING_SERVICE_NAME, 
						    &ObjectsDatabaseNode::graspPlanningCB, this);
  }

  ~ObjectsDatabaseNode()
  {
    delete database_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_database_node");
  ObjectsDatabaseNode node;
  ros::spin();
  return 0;  
}
