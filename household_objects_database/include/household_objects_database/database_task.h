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

#ifndef _DATABASE_TASK_H_
#define _DATABASE_TASK_H_

#include <database_interface/db_class.h>

namespace household_objects_database {

//! The ID of a task to be retrieved from the special get_mark_next_task() function 
/*! This function will atomically acquire an ID from a task set to TO_RUN and mark it
  as RUNNING. After that is done, we can use the class below this one to recover the rest of
  the information about the task.
 */
class DatabaseTaskID : public database_interface::DBClass
{
 public:
  database_interface::DBField<int> id_;
  
 DatabaseTaskID() :
  id_(database_interface::DBFieldBase::TEXT, this, "get_mark_next_task", "get_mark_next_task()", false)
    {
      primary_key_field_ = &id_;
      id_.setWriteToDatabase(false);
    }
};

class DatabaseTask : public database_interface::DBClass
{
 private:
 public:
  //primary key
  database_interface::DBField<int> id_;
  //other fields
  database_interface::DBField<int> type_;
  database_interface::DBField<int> scaled_model_id_;
  database_interface::DBField<std::string> hand_name_;
  database_interface::DBField<std::string> outcome_name_;
  database_interface::DBField<int> time_;

  DatabaseTask() : 
    id_(database_interface::DBFieldBase::TEXT, this, "task_id", "task", true),
    type_(database_interface::DBFieldBase::TEXT, this, "task_type", "task", true),
    scaled_model_id_(database_interface::DBFieldBase::TEXT, this, "scaled_model_id", "task", true),
    hand_name_(database_interface::DBFieldBase::TEXT, this, "hand_name", "task", true),
    outcome_name_(database_interface::DBFieldBase::TEXT, this, "task_outcome_name", "task", true),
    time_(database_interface::DBFieldBase::TEXT, this, "task_time", "task", true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&type_);
    fields_.push_back(&scaled_model_id_);
    fields_.push_back(&hand_name_);
    fields_.push_back(&outcome_name_);
    fields_.push_back(&time_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    id_.setSequenceName("task_id_seq");
    id_.setWriteToDatabase(false);
  }
};

} //namespace

#endif
