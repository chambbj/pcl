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

#include <algorithm>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/normal_distribution.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/octree/octree_search.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::common;
using namespace pcl::console;
using namespace pcl::octree;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef const Cloud::ConstPtr ConstCloudPtr;

float default_radius = 0.15f;
double default_stddev = 0.15;

struct density_point_idx
{
    boost::uint32_t density;
    boost::uint32_t point_idx;

    density_point_idx (boost::uint32_t density_, 
                       boost::uint32_t point_idx_) 
        : density(density_)
        , point_idx(point_idx_) {};
    bool operator < (const density_point_idx &p) const
    {
        return (density < p.density);
    }
};

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s samples.pcd look.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X = the radius to be used (default: ");
  print_value ("%f", default_radius); print_info (")\n");
  print_info ("                     -stddev X = the standard deviation to be used (default: ");
  print_value ("%f", default_stddev); print_info (")\n");
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
compute (ConstCloudPtr &samples, ConstCloudPtr &look, Cloud &output, float radius, double stddev)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  OctreePointCloudSearch<PointType>::Ptr tree (new OctreePointCloudSearch<PointType> (1.0f));
  tree->setInputCloud (look);
  tree->addPointsFromInputCloud ();

  std::vector<density_point_idx> density;
  density.reserve (samples->points.size ());
  density.resize (samples->points.size (), density_point_idx (0, 0));

  print_info ("compute density\n");

  for (boost::int32_t p_idx = 0; p_idx < samples->points.size(); ++p_idx)
  {
    std::vector<int> pt_indices;
    std::vector<float> sqr_distances;

    tree->radiusSearch (samples->points[p_idx], radius, pt_indices, sqr_distances);
    density[p_idx].density = (pt_indices.size ()+1);
    density[p_idx].point_idx = p_idx;
  }

  print_info ("sort density\n");

  std::sort (density.begin (), density.end (), std::less<density_point_idx> ());

  std::vector<float> cdf;
  cdf.reserve (density.size ());
  cdf.resize (density.size ());

  print_info ("cumulative density\n");

  cdf[0] = static_cast<float> (density[0].density);
  for (boost::int32_t p_idx = 1; p_idx < density.size (); ++p_idx)
  {
    cdf[p_idx] = cdf[p_idx-1] + static_cast<float> (density[p_idx].density);
  }

  float denom = cdf[cdf.size ()-1];

  print_info ("normalize cumulative density\n");

  for (boost::int32_t p_idx = 0; p_idx < density.size (); ++p_idx)
  {
    cdf[p_idx] /= denom;
  }

  print_info ("generate next stage samples\n");

  boost::random::mt19937 eng;
  boost::random::uniform_real_distribution<> dist;
  boost::random::normal_distribution<> nor (0.0, stddev);

  for ( boost::int32_t p_idx = 0; p_idx < samples->points.size(); ++p_idx )
  {
    print_info ("point idx "); print_value ("%d; ", p_idx);

    float rn = dist (eng);
    print_info ("random value "); print_value ("%f; ", rn);

    // generate random number and find the index in density that matches
    // density[index].idx is the point in the cloud that this corresponds to
    // cloud_f->points[p_idx] = cloud->points[density[index].idx];
    // perturbate the point by a small amount
    //int idx = 0;
    //print_info ("upper_bound at position "); print_value ("%d\n", idx);
    //print_info ("density is "); print_value ("%d\n", density[idx].density );
    //print_info ("point index is "); print_value ("%d\n", density[idx].point_idx);
    //print_info ("cdf is "); print_value ("%f\n", cdf[idx]);

    std::vector<float>::iterator up;
    up = std::upper_bound (cdf.begin(), cdf.end(), rn);
    int idx = up - cdf.begin() - 1;
    if (idx < 0) idx = 0; // ick
    print_info ("idx "); print_value ("%d; ", idx);

    //print_info ("upper_bound at position "); print_value ("%d\n", idx);
    //print_info ("density is "); print_value ("%d\n", density[idx].density );
    //print_info ("point index is "); print_value ("%d\n", density[idx].point_idx);
    //print_info ("cdf is "); print_value ("%f\n", cdf[idx]);

    PointType point = samples->points[density[idx].point_idx];
    print_info ("noise "); print_value ("%f\n", nor (eng));
    point.x += nor (eng);
    point.y += nor (eng);
    point.z += nor (eng);

    output.points[p_idx] = point;

    //up = std::upper_bound (cdf.begin(), cdf.end(), 0.5f);
    //idx = up - cdf.begin() - 1;
    //print_info ("upper_bound at position "); print_value ("%d\n", idx);
    //print_info ("density is "); print_value ("%d\n", density[idx].density );
    //print_info ("point index is "); print_value ("%d\n", density[idx].point_idx);
    //print_info ("cdf is "); print_value ("%f\n", cdf[idx]);

    //up = std::upper_bound (cdf.begin(), cdf.end(), 1.0f);
    //idx = up - cdf.begin() - 1;
    //print_info ("upper_bound at position "); print_value ("%d\n", idx);
    //print_info ("density is "); print_value ("%d\n", density[idx].density );
    //print_info ("point index is "); print_value ("%d\n", density[idx].point_idx);
    //print_info ("cdf is "); print_value ("%f\n", cdf[idx]);
  }

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const Cloud &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (find_switch (argc, argv, "-h"))
  {
    printHelp (argc, argv);
    return (0);
  }

  print_info ("Perform condensation (one iteration). For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  float radius = default_radius;
  double stddev = default_stddev;
  parse_argument (argc, argv, "-radius", radius);
  parse_argument (argc, argv, "-stddev", stddev);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 3)
  {
    print_error ("Need two input PCD files and one output PCD file to continue.\n");
    return (-1);
  }

  // Load the first file (samples)
  Cloud::Ptr samples (new Cloud);
  if (!loadCloud (argv[p_file_indices[0]], *samples))
    return (-1);

  // Load the second file (look)
  Cloud::Ptr look (new Cloud);
  if (!loadCloud (argv[p_file_indices[1]], *look))
    return (-1);

  // Perform the feature estimation
  Cloud output;
  copyPointCloud<PointType, PointType> (*samples, output);
  compute (samples, look, output, radius, stddev);

  // Save into the second file
  saveCloud (argv[p_file_indices[2]], output);
}
