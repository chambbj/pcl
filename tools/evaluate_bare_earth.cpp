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
  print_error ("Syntax is: %s original.pcd reference.pcd filtered.pcd tolerance\n", argv[0]);
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
compute (Cloud &original, Cloud &reference, Cloud &filtered, float tol)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  // create trees for reference bare earth (a) and reference object (b) clouds
  pcl::search::KdTree<PointType> tree_original;
  pcl::search::KdTree<PointType> tree_reference;
  tree_original.setInputCloud (original.makeShared ());
  tree_reference.setInputCloud (reference.makeShared ());

  int a0, a, b, c, d, e, j;
  a0 = a = b = c = d = j = 0;
  e = original.points.size ();

  //float tol = 0.1;
  float sqr_tol = std::pow (tol, 2);

  print_info ("tolerance: "); print_value ("%f", tol);
  print_info (" and squared tolerance: "); print_value ("%f\n", sqr_tol);

  // iterate through filtered bare earth (c)
  for (int i = 0; i < filtered.points.size (); ++i)
  {
    // find nearest neighbor of filtered bare earth in original
    std::vector<int> orig_indices (1);
    std::vector<float> orig_sqr_distances (1);
    tree_original.nearestKSearch (filtered.points[i], 1, orig_indices, orig_sqr_distances);

    // find nearest neighbor of filtered bare earth in reference
    std::vector<int> ref_indices (1);
    std::vector<float> ref_sqr_distances (1);
    tree_reference.nearestKSearch (filtered.points[i], 1, ref_indices, ref_sqr_distances);

    if (orig_sqr_distances[0] < sqr_tol) // && orig_sqr_distances[0] < ref_sqr_distances[0])
      a0++;
    else if (ref_sqr_distances[0] < sqr_tol) // && ref_sqr_distances[0] < orig_sqr_distances[0])
      a++;
    /*
    else
    {
      j++;
      std::cerr << "Filtered bare earth point " << i << " = " << filtered.points[0] << std::endl;
      std::cerr << "Reference bare earth point " << orig_indices[0] << " = " << original.points[orig_indices[0]]
        << ", distances = " << orig_sqr_distances[0] << std::endl;
      std::cerr << "Reference object point " << ref_indices[0] << " = " << reference.points[ref_indices[0]]
        << ", distances = " << ref_sqr_distances[0] << std::endl;
    }
    */
  }

  /*
  // iterate through filtered object (d)
  for (int i = 0; i < cloud_d.points.size (); ++i)
  {
    // find nearest neighbor of filtered object in reference bare earth
    std::vector<int> ad_indices (1);
    std::vector<float> ad_sqr_distances (1);
    int nk_ad = tree_original.nearestKSearch (cloud_d.points[i], 1, ad_indices, ad_sqr_distances);

    // find nearest neighbor of filtered object in reference object
    std::vector<int> bd_indices (1);
    std::vector<float> bd_sqr_distances (1);
    int nk_bd = tree_reference.nearestKSearch (cloud_d.points[i], 1, bd_indices, bd_sqr_distances);

    if (ad_sqr_distances[0] < sqr_tol && ad_sqr_distances[0] < bd_sqr_distances[0])
      b++;
    else if (bd_sqr_distances[0] < sqr_tol && bd_sqr_distances[0] < ad_sqr_distances[0])
      d++;
    else
    {
      j++;
      std::cerr << std::setprecision(8);
      std::cerr << std::fixed;
    //  std::cerr << "Filtered object point " << i << " = " << cloud_d.points[i] << std::endl;
//      std::cerr << "Reference bare earth point " << ad_indices[0] << " = " << original.points[ad_indices[0]]
  //      << ", distances = " << ad_sqr_distances[0] << std::endl;
    //  std::cerr << "Reference object point " << bd_indices[0] << " = " << reference.points[bd_indices[0]]
      //  << ", distances = " << bd_sqr_distances[0] << std::endl;
//      std::cerr << pcl::squaredEuclideanDistance (cloud_d.points[i], original.points[ad_indices[0]]) << std::endl;
  //    std::cerr << pcl::squaredEuclideanDistance (cloud_d.points[i], reference.points[bd_indices[0]]) << std::endl;
      std::cerr << ad_sqr_distances[0] << ", " << bd_sqr_distances[0] << ", " << sqr_tol << std::endl;
    }
  }
  */

  b = e - a;
  c = a0 - a;
  d = e - c;

  //int e = a + b + c + d;
  float f = static_cast<float> (a+b) / static_cast<float> (e);
  float g = static_cast<float> (c+d) / static_cast<float> (e);
  float h = static_cast<float> (a+c) / static_cast<float> (e);
  float i = static_cast<float> (b+d) / static_cast<float> (e);
  float k = static_cast<float> (b) / static_cast<float> (c);
  float pd = static_cast<float> (a+d) / static_cast<float> (e);
  float pfa = static_cast<float> (b+c) / static_cast<float> (e);

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
  print_info ("[Pd: "); print_value ("%0.2f%%", pd*100.0f);
  print_info (", Pfa: "); print_value ("%0.2f%%", pfa*100.0f);
  print_info (" ]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Evaluate bare earth performance. For more information, use: %s -h\n", argv[0]);

  if (argc < 6)
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
  Cloud::Ptr original (new Cloud);
  if (!loadCloud (argv[p_file_indices[0]], *original))
    return (-1);

  // Load the second file
  Cloud::Ptr reference (new Cloud);
  if (!loadCloud (argv[p_file_indices[1]], *reference))
    return (-1);

  // Load the third file
  Cloud::Ptr filtered (new Cloud);
  if (!loadCloud (argv[p_file_indices[2]], *filtered))
    return (-1);

  // Load the fourth file
  Cloud::Ptr cloud_d (new Cloud);
  if (!loadCloud (argv[p_file_indices[3]], *cloud_d))
    return (-1);

  // Compute the Hausdorff distance
  compute (*original, *reference, *filtered, std::atof (argv[5]));
}

