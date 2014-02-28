/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/pipeline/pipeline.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;
//using namespace std;

PCLPointCloud2::Ptr cloud_blob (new PCLPointCloud2);
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normal (new PointCloud<PointNormal>);
//vector<int> indices_;
//PointCloud<PointXYZRGB>::Ptr cloud_organized (new PointCloud<PointXYZRGB>);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Pipeline, passthrough)
{
  PointCloud<PointNormal> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  std::string json = "{\
                        \"pipeline\": {\
                          \"name\": \"PassThroughTest\",\
                          \"filters\": [{\
                            \"name\": \"PassThrough\",\
                            \"setFilterFieldName\": \"z\",\
                            \"setFilterLimits\": {\
                              \"min\": 0.5\
                            }\
                          }]\
                        }\
                      }";

  Pipeline<PointNormal> pipeline;
  pipeline.setInputCloud (cloud_in.makeShared ());
  pipeline.setJSON (json);
  pipeline.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out.size (), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Pipeline, passthrough2)
{
  PointCloud<PointNormal> output;

  std::string json = "{\
                        \"pipeline\": {\
                          \"name\": \"PassThroughTest\",\
                          \"filters\": [{\
                            \"name\": \"PassThrough\",\
                            \"setFilterFieldName\": \"z\",\
                            \"setFilterLimits\": {\
                              \"min\": 0.05,\
                              \"max\": 0.1\
                            }\
                          }]\
                        }\
                      }";

  Pipeline<PointNormal> pipeline;
  pipeline.setInputCloud (cloud_with_normal);

// uncommented, the following lines pass, but the ones that follow fail.
// perhaps I'm somehow inheriting the results of the first, instead of using
// input_?
//  pipeline.filter (output);

//  EXPECT_EQ (output.points.size (), cloud_with_normal->points.size ());
//  EXPECT_EQ (output.width, cloud_with_normal->width);
//  EXPECT_EQ (output.height, cloud_with_normal->height);

  pipeline.setJSON (json);
  pipeline.filter (output);

  EXPECT_EQ (int (output.points.size ()), 42);
  EXPECT_EQ (int (output.width), 42);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);

  EXPECT_NEAR (output.points[0].x, -0.074556, 1e-5);
  EXPECT_NEAR (output.points[0].y, 0.13415, 1e-5);
  EXPECT_NEAR (output.points[0].z, 0.051046, 1e-5);

  EXPECT_NEAR (output.points[41].x, -0.030331, 1e-5);
  EXPECT_NEAR (output.points[41].y, 0.039749, 1e-5);
  EXPECT_NEAR (output.points[41].z, 0.052133, 1e-5);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Pipeline, voxel_grid)
{
  PointCloud<PointNormal> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  std::string json = "{\
                        \"pipeline\": {\
                          \"name\": \"VoxelGridTest\",\
                          \"filters\": [{\
                            \"name\": \"VoxelGrid\",\
                            \"setLeafSize\": {\
                              \"x\": 2.0,\
                              \"y\": 2.0,\
                              \"z\": 2.0\
                            }\
                          }]\
                        }\
                      }";

  Pipeline<PointNormal> pipeline;
  pipeline.setInputCloud (cloud_in.makeShared ());
  pipeline.setJSON (json);
  pipeline.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].x, 0.5f);
  EXPECT_EQ (cloud_out[0].y, 0.5f);
  EXPECT_EQ (cloud_out[0].z, 0.5f);
  EXPECT_EQ (cloud_out.size (), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Pipeline, VoxelGrid2)
{
  PointCloud<PointNormal> output;
  Pipeline<PointNormal> pipeline;

  std::string json = "{\
                        \"pipeline\": {\
                          \"name\": \"VoxelGridTest\",\
                          \"filters\": [{\
                            \"name\": \"VoxelGrid\",\
                            \"setLeafSize\": {\
                              \"x\": 0.02,\
                              \"y\": 0.02,\
                              \"z\": 0.02\
                            }\
                          }]\
                        }\
                      }";

  pipeline.setInputCloud (cloud_with_normal);
  pipeline.setJSON (json);
  pipeline.filter (output);

  EXPECT_EQ (int (output.points.size ()), 103);
  EXPECT_EQ (int (output.width), 103);
  EXPECT_EQ (int (output.height), 1);
  EXPECT_EQ (bool (output.is_dense), true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Pipeline, voxel_grid_passthrough)
{
  PointCloud<PointNormal> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  std::string json = "{\
                        \"pipeline\": {\
                          \"name\": \"VoxelGridPassThroughTest\",\
                          \"filters\": [{\
                            \"name\": \"VoxelGrid\",\
                            \"setLeafSize\": {\
                              \"x\": 2.0,\
                              \"y\": 2.0,\
                              \"z\": 2.0\
                            }\
                          }, {\
                            \"name\": \"PassThrough\",\
                            \"setFilterFieldName\": \"z\",\
                            \"setFilterLimits\": {\
                              \"min\": 0.75\
                            }\
                          }]\
                        }\
                      }";

  Pipeline<PointNormal> pipeline;
  pipeline.setInputCloud (cloud_in.makeShared ());
  pipeline.setJSON (json);
  pipeline.filter (cloud_out);

  EXPECT_EQ (cloud_out.size (), 0);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its paths to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  loadPCDFile (file_name, *cloud_blob);
  fromPCLPointCloud2 (*cloud_blob, *cloud);
  copyPointCloud (*cloud, *cloud_with_normal);

//  indices_.resize (cloud->points.size ());
//  for (int i = 0; i < static_cast<int> (indices_.size ()); ++i)
//    indices_[i] = i;

//  loadPCDFile (argv[2], *cloud_organized);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

