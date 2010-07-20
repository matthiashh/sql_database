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

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include "household_objects_database/objects_database.h"
#include "household_objects_database/database_original_model.h"
#include "household_objects_database/database_scaled_model.h"

void usage()
{
  std::cerr << "Usage: insert_model geometry_file [thumbnail_file]\n";
}

std::string getNonEmptyString(const std::string &display_name)
{
  std::string ret_string;
  while(1)
  {
    std::cout << "Enter " << display_name << ": ";
    std::getline(std::cin, ret_string);
    if (!ret_string.empty()) break;
    std::cout << "Non-empty string required \n";
  }
  return ret_string;
}

/*! Reads a file into a binary chunk */
bool readBinaryFile(std::string filename, std::vector<char> &binary_chunk)
{
  std::ifstream geometry_file(filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
  if (!geometry_file.is_open())
  {
    std::cerr << "Failed to open geometry file " << filename << "\n";
    return false;
  } 
  std::ifstream::pos_type size = geometry_file.tellg();
  geometry_file.seekg(0, std::ios::beg);
  binary_chunk.resize(size);
  geometry_file.read(&binary_chunk[0], size);
  if (geometry_file.fail())
  {
    std::cerr << "Failed to read binary geometry from file " << geometry_file << "\n";
    geometry_file.close();
    return false;
  }
  //std::cerr << "Read " << size << " bytes from geometry file\n";
  geometry_file.close();
  return true;
}

bool copyFile(std::string old_path, std::string new_root, std::string &filename)
{
  //get just the filename part of the original file
  size_t slash_position = old_path.find_last_of('/');
  if (slash_position == std::string::npos) 
  {
    //no slash
    filename = old_path;
  } 
  else if (slash_position >= old_path.size() - 1)
  {
    //slash is the last character
    std::cerr << "Failed to parse input filename: " << old_path << "\n";
    return false;
  }
  else
  {
    //skip the last slash
    filename = old_path.substr(slash_position+1, std::string::npos);
  }
  std::string new_path(new_root);
  if (new_path.at(new_path.size() - 1) != '/') 
  {
    new_path.append("//");
  }
  new_path.append(filename);
  if ( boost::filesystem::exists(new_path) )
  {
    std::cerr << "File " << new_path << " already exists; skipping copy.\n";
    return true;
  }  
  boost::filesystem::copy_file(old_path, new_path);
  if ( !boost::filesystem::exists(new_path) )
  {
    std::cerr << "Failed to copy file " << filename << " to " << new_path << "\n";
    return false;
  }
  return true;
}

std::string getString(const std::string &display_name)
{
  std::string ret_string;
  std::cout << "Enter " << display_name << " (leave empty for none):";
  std::getline(std::cin, ret_string);
  return ret_string;
}

int main(int argc, char **argv)
{
  //connect to database
  //hard-coded for now
  household_objects_database::ObjectsDatabase database("bcj", "5432", "matei", "willow", "data_wg_test");
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect";
    return -1;
  }

  //parse the input
  //first argument is always the filename which must exist
  if (argc < 2) 
  {
    usage();
    return -1;
  }

  //check that the file exists
  std::string geometry_filename(argv[1]);
  if ( !boost::filesystem::exists(geometry_filename) )
  {
    std::cerr << "File " << geometry_filename << " not found\n";
    return -1;
  }
  
  //the original model we will insert
  household_objects_database::DatabaseOriginalModel original_model;

  original_model.source_.get() = getNonEmptyString("object source");
  original_model.acquisition_method_.get() = getNonEmptyString("acquisition method");
  original_model.maker_.get() = getNonEmptyString("object maker");
  original_model.model_.get() = getNonEmptyString("object model");

  original_model.barcode_.get() = getString("object barcode");
  if (!original_model.barcode_.get().empty())
  {
    original_model.barcode_.setWriteToDatabase(true);
  }
  original_model.description_.get() = getString("object description");
  if (!original_model.description_.get().empty()) 
  {
    original_model.description_.setWriteToDatabase(true);
  }
  std::vector<std::string> tags;
  std::cout << "Enter tags one at a time; enter an empty line when done\n";
  while(1)
  {
    std::string tag;
    std::getline(std::cin,tag);
    if (!tag.empty()) tags.push_back(tag);
    else break;
  }
  if (!tags.empty())
  {
    original_model.tags_.get() = tags;
    original_model.tags_.setWriteToDatabase(true);
  }

  //check thumbnail file, second argument (optional)
  std::string thumbnail_filename;
  if (argc >= 3)
  {
    thumbnail_filename = argv[2];
    original_model.geometry_thumbnail_path_.get() = thumbnail_filename;
    std::cerr << "Thumbnail path: " << thumbnail_filename << "\n";
    if (!boost::filesystem::exists( original_model.geometry_thumbnail_path_.get() ))
    {
      std::cout << "Thumbnail file does not exist\n";
      return -1;
    }
  }

  //load the geometry binary file 
  if ( !readBinaryFile(geometry_filename, original_model.geometry_binary_ply_.get()) ) 
  {
    std::cout << "Failed to read geometry file " << geometry_filename << "\n";
    return -1;
  }
  //load the thumbnail binary file
  if (!thumbnail_filename.empty())
  {
    if ( !readBinaryFile( thumbnail_filename, original_model.geometry_thumbnail_binary_png_.get()) ) 
    {
      std::cout << "Failed to read thumbnail file " << thumbnail_filename << "\n";
      return -1;
    }
  }

  //copy the file(s) over to the model root
  std::string model_root;
  if (!database.getModelRoot(model_root) || model_root.empty())
  {
    std::cerr << "Failed to get model root from database\n";
    return -1;
  }
  std::string filename;
  if (!copyFile(geometry_filename, model_root, filename)) return -1;
  original_model.geometry_path_.get() = filename;
  original_model.geometry_path_.setWriteToDatabase(true);
  if(!thumbnail_filename.empty())
  {
    if (!copyFile(thumbnail_filename, model_root, filename)) return -1;
    original_model.geometry_thumbnail_path_.get() = filename;
    original_model.geometry_thumbnail_path_.setWriteToDatabase(true);
  }
  
  //insert the original model into the database
  if (!database.insertIntoDatabase(&original_model))
  {
    std::cerr << "Failed to insert original model in database\n";
    return -1;
  }
  int original_model_id = original_model.id_.get();
  //std::cout << "Original model inserted. Id: " << original_model_id << "\n";

  //write the binary parts
  if (!database.saveToDatabase(&original_model.geometry_binary_ply_))
  {
    std::cerr << "Failed to write binary ply geometry to database\n";
    return -1;
  }
  if (!thumbnail_filename.empty())
  {
    if (!database.saveToDatabase(&original_model.geometry_thumbnail_binary_png_))
    {
      std::cerr << "Failed to write binary thumbnail to database\n";
      return -1;
    }
  }

  // insert a scaled model at range 1.0
  household_objects_database::DatabaseScaledModel scaled_model;
  scaled_model.original_model_id_.get() = original_model_id;
  scaled_model.scale_.get() = 1.0;
  if (!database.insertIntoDatabase(&scaled_model))
  {
    std::cerr << "Failed to insert scaled model in database\n";
    return -1;
  }
  //std::cout << "Scaled model inserted. Id: " << scaled_model.id_.get() << "\n";


  //test the binary entry
  /*
  household_objects_database::DatabaseOriginalModel test_model;
  test_model.id_.get() = original_model_id;
  if (!database.loadFromDatabase(&test_model.geometry_binary_ply_))
  {
    std::cerr << "Failed to load binary geometry in test\n";
    return -1;
  }
  std::ofstream test_file("test_file", std::ofstream::binary);
  test_file.write( &(test_model.geometry_binary_ply_.get().at(0)), test_model.geometry_binary_ply_.get().size() );
  test_file.close();
  std::cout << "Wrote test binary file\n";
  */

  std::cerr << "Insertion succeeded\n";
  return 0;
}
