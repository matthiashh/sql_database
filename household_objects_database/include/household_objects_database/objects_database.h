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

#ifndef _OBJECTS_DATABASE_H_
#define _OBJECTS_DATABASE_H_

//for ROS error messages
#include <ros/ros.h>

#include <database_interface/postgresql_database_interface.h>

#include "household_objects_database/database_original_model.h"
#include "household_objects_database/database_scaled_model.h"
#include "household_objects_database/database_grasp.h"

namespace household_objects_database {

class DatabaseTask;

//! A slight specialization of the general database interface with a few convenience functions added
class ObjectsDatabase : public database_interface::PostgresqlDatabaseInterface
{
 public:
  //! Attempts to connect to the specified database
  ObjectsDatabase(std::string host, std::string port, std::string user,
		  std::string password, std::string dbname) 
    : PostgresqlDatabaseInterface(host, port, user, password, dbname) {}

  //! Attempts to connect to the specified database
  ObjectsDatabase(const database_interface::PostgresqlDatabaseConfig &config) 
    : PostgresqlDatabaseInterface(config) {}

  //! Just a stub for now
  ~ObjectsDatabase() {}

  //! Acquires the next experiment to be executed from the list of tasks in the database
  /*! Also marks it as RUNNING in an atomic fashion, so that it is not acquired by 
   another process.*/
  virtual bool acquireNextTask(boost::ptr_vector<DatabaseTask> &task);

  //------- helper functions wrapped around the general versions for convenience -------
  //----------------- or for cases where where_clauses are needed ----------------------

  //! Gets a list of all the original models in the database
  bool getOriginalModelsList(boost::ptr_vector<DatabaseOriginalModel> &models) const
  {
    DatabaseOriginalModel example;
    return getList<DatabaseOriginalModel>(models, example, "");
  }

  //! Gets a list of all the scaled models in the database
  bool getScaledModelsList(boost::ptr_vector<DatabaseScaledModel> &models) const
  {
    DatabaseScaledModel example;
    return getList<DatabaseScaledModel>(models, example, "");
  }

  //! Gets a list of scaled models based on acquisition method
  bool getScaledModelsByAcquisition(boost::ptr_vector<DatabaseScaledModel> &models, 
				    std::string acquisition_method) const
  {
    std::string where_clause("acquisition_method_name='" + acquisition_method + "'");
    DatabaseScaledModel example;
    //this should be set by default, but let's make sure again
    example.acquisition_method_.setReadFromDatabase(true);
    return getList<DatabaseScaledModel>(models, example, where_clause);
  }

  //! Gets a list of scaled models with "experiment_set" true
  virtual bool getScaledModelsExperimentSet(boost::ptr_vector<DatabaseScaledModel> &models) const
  {
    std::string where_clause("original_model_experiment_set='TRUE'");
    DatabaseScaledModel example;
    return getList<DatabaseScaledModel>(models, example, where_clause);
  }

  //! Gets a list of scaled models with "reduced_experiment_set" true
  virtual bool getScaledModelsReducedExperimentSet(boost::ptr_vector<DatabaseScaledModel> &models) const
  {
    std::string where_clause("original_model_reduced_experiment_set='TRUE'");
    DatabaseScaledModel example;
    return getList<DatabaseScaledModel>(models, example, where_clause);
  }

  //! Gets a list of scaled models with "icra_experiment_set" true
  virtual bool getScaledModelsIcraExperimentSet(boost::ptr_vector<DatabaseScaledModel> &models) const
  {
    std::string where_clause("original_model_icra_experiment_set='TRUE'");
    DatabaseScaledModel example;
    return getList<DatabaseScaledModel>(models, example, where_clause);
  }

  //! Returns the number of original models in the database
  bool getNumOriginalModels(int &num_models) const
  {
    DatabaseOriginalModel example;
    return countList(&example, num_models, "");
  }

  //! Returns the path that geometry paths are relative to
  bool getModelRoot(std::string& root) const
  {
    return getVariable("'MODEL_ROOT'", root);
  }

  //! Gets a list of all models with the requested tags in the database
  bool getModelsListByTags(boost::ptr_vector<DatabaseOriginalModel> &models, 
			   std::vector<std::string> tags) const
  {
    DatabaseOriginalModel example;
    std::string where_clause("(");
    for (size_t i=0; i<tags.size(); i++)
    {
      if (i!=0) where_clause += "AND ";
      where_clause += "'" + tags[i] + "' = ANY (original_model_tags)";
    }
    where_clause += ")";
    return getList<DatabaseOriginalModel>(models, example, where_clause);    
  }

  //! Gets the list of all the grasps for a scaled model id
  bool getGrasps(int scaled_model_id, std::string hand_name, boost::ptr_vector<DatabaseGrasp> &grasps) const
  {
    DatabaseGrasp example;
    std::stringstream id;
    id << scaled_model_id;
    std::string where_clause("scaled_model_id=" + id.str() + 
			     " AND hand_name='" + hand_name + "'");
    return getList<DatabaseGrasp>(grasps, example, where_clause);
  }

  //! Gets the list of only those grasps that are cluster reps a database model
  bool getClusterRepGrasps(int scaled_model_id, std::string hand_name, boost::ptr_vector<DatabaseGrasp> &grasps) const
  {
    DatabaseGrasp example;
    std::stringstream id;
    id << scaled_model_id;
    std::string where_clause("scaled_model_id=" + id.str() + 
			     " AND hand_name='" + hand_name + "'" + 
			     " AND grasp_cluster_rep=true");
    return getList<DatabaseGrasp>(grasps, example, where_clause);
  }
};

}//namespace

#endif
