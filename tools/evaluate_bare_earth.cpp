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
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s cloud_a.pcd cloud_b.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, Cloud &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (Cloud &cloud_a, Cloud &cloud_b, Cloud &cloud_c, Cloud &cloud_d)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  // create trees for reference bare earth (a) and reference object (b) clouds
  pcl::search::KdTree<PointType> tree_a;
  pcl::search::KdTree<PointType> tree_b;
  tree_a.setInputCloud (cloud_a.makeShared ());
  tree_b.setInputCloud (cloud_b.makeShared ());

  int a, b, c, d, j;
  a = b = c = d = j = 0;

  float tol = 1e-2;
  float sqr_tol = std::pow (tol, 2);

  print_info ("tolerance: "); print_value ("%f", tol);
  print_info (" and squared tolerance: "); print_value ("%f\n", sqr_tol);

  // iterate through filtered bare earth (c)
  for (int i = 0; i < cloud_c.points.size (); ++i)
  {
    // find nearest neighbor of filtered bare earth in reference bare earth
    std::vector<int> ac_indices (1);
    std::vector<float> ac_sqr_distances (1);
    tree_a.nearestKSearch (cloud_c.points[i], 1, ac_indices, ac_sqr_distances);

    // find nearest neighbor of filtered bare earth in reference object
    std::vector<int> bc_indices (1);
    std::vector<float> bc_sqr_distances (1);
    tree_b.nearestKSearch (cloud_c.points[i], 1, bc_indices, bc_sqr_distances);

    if (ac_sqr_distances[0] < sqr_tol && ac_sqr_distances[0] < bc_sqr_distances[0])
      a++;
    else if (bc_sqr_distances[0] < sqr_tol && bc_sqr_distances[0] < ac_sqr_distances[0])
      c++;
    else
    {
      j++;
      std::cerr << "Filtered bare earth point " << i << " = " << cloud_c.points[0] << std::endl;
      std::cerr << "Reference bare earth point " << ac_indices[0] << " = " << cloud_a.points[ac_indices[0]]
        << ", distances = " << ac_sqr_distances[0] << std::endl;
      std::cerr << "Reference object point " << bc_indices[0] << " = " << cloud_b.points[bc_indices[0]]
        << ", distances = " << bc_sqr_distances[0] << std::endl;
    }
  }

  // iterate through filtered object (d)
  for (int i = 0; i < cloud_d.points.size (); ++i)
  {
    // find nearest neighbor of filtered object in reference bare earth
    std::vector<int> ad_indices (1);
    std::vector<float> ad_sqr_distances (1);
    int nk_ad = tree_a.nearestKSearch (cloud_d.points[i], 1, ad_indices, ad_sqr_distances);

    // find nearest neighbor of filtered object in reference object
    std::vector<int> bd_indices (1);
    std::vector<float> bd_sqr_distances (1);
    int nk_bd = tree_b.nearestKSearch (cloud_d.points[i], 1, bd_indices, bd_sqr_distances);

    if (ad_sqr_distances[0] < sqr_tol && ad_sqr_distances[0] < bd_sqr_distances[0])
      b++;
    else if (bd_sqr_distances[0] < sqr_tol && bd_sqr_distances[0] < ad_sqr_distances[0])
      d++;
    else
    {
      j++;
      std::cerr << std::setprecision(8);
      std::cerr << std::fixed;
      std::cerr << "Filtered object point " << i << " = " << cloud_d.points[i] << std::endl;
      std::cerr << "Reference bare earth point " << ad_indices[0] << " = " << cloud_a.points[ad_indices[0]]
        << ", distances = " << ad_sqr_distances[0] << std::endl;
      std::cerr << "Reference object point " << bd_indices[0] << " = " << cloud_b.points[bd_indices[0]]
        << ", distances = " << bd_sqr_distances[0] << std::endl;
      std::cerr << pcl::squaredEuclideanDistance (cloud_d.points[i], cloud_a.points[ad_indices[0]]) << std::endl;
      std::cerr << pcl::squaredEuclideanDistance (cloud_d.points[i], cloud_b.points[bd_indices[0]]) << std::endl;
    }
  }

  int e = a + b + c + d;
  float f = static_cast<float> (a+b) / static_cast<float> (e);
  float g = static_cast<float> (c+d) / static_cast<float> (e);
  float h = static_cast<float> (a+c) / static_cast<float> (e);
  float i = static_cast<float> (b+d) / static_cast<float> (e);
  float k = static_cast<float> (b) / static_cast<float> (c);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  print_info ("a: "); print_value ("%d", a);
  print_info (", b: "); print_value ("%d", b);
  print_info (", c: "); print_value ("%d", c);
  print_info (", d: "); print_value ("%d", d);
  print_info (", e: "); print_value ("%d", e);
  print_info (", f: "); print_value ("%0.2f%%", f*100.0f);
  print_info (", g: "); print_value ("%0.2f%%", g*100.0f);
  print_info (", h: "); print_value ("%0.2f%%", h*100.0f);
  print_info (", i: "); print_value ("%0.2f%%", i*100.0f);
  print_info (", j: "); print_value ("%d", j);
  print_info (", k: "); print_value ("%0.2f", k);
  print_info (" ]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Evaluate bare earth performance. For more information, use: %s -h\n", argv[0]);

  if (argc < 5)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 4)
  {
    print_error ("Need two reference PCD files (bare earth and object) and two filtered PCD files (bare earth and object) to evaluate bare earth.\n");
    return (-1);
  }

  // Load the first file
  Cloud::Ptr cloud_a (new Cloud);
  if (!loadCloud (argv[p_file_indices[0]], *cloud_a))
    return (-1);

  // Load the second file
  Cloud::Ptr cloud_b (new Cloud);
  if (!loadCloud (argv[p_file_indices[1]], *cloud_b))
    return (-1);

  // Load the third file
  Cloud::Ptr cloud_c (new Cloud);
  if (!loadCloud (argv[p_file_indices[2]], *cloud_c))
    return (-1);

  // Load the fourth file
  Cloud::Ptr cloud_d (new Cloud);
  if (!loadCloud (argv[p_file_indices[3]], *cloud_d))
    return (-1);

  // Compute the Hausdorff distance
  compute (*cloud_a, *cloud_b, *cloud_c, *cloud_d);
}

