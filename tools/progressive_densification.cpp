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

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include <Eigen/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
namespace ba = boost::accumulators;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef const Cloud::ConstPtr ConstCloudPtr;

float default_resolution = 10.0f;
float default_dist_thresh = 1.5f;
float default_angle_thresh = 6.0f;
int default_verbosity_level = 3;
int default_max_iters = 3;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -resolution X = resolution to compute grid minimums (default: ");
  print_value ("%f", default_resolution);
  print_info (")\n");
  print_info ("                     -dist_thresh X = distance threshold for densification (default: ");
  print_value ("%f", default_dist_thresh);
  print_info (")\n");
  print_info ("                     -angle_thresh X = angle threshold for densification (default: ");
  print_value ("%f", default_angle_thresh);
  print_info (")\n");
  print_info ("                     -max_iters X = maximum number of iterations (default: ");
  print_value ("%d", default_max_iters);
  print_info (")\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
  print_info ("                     -verbosity X = verbosity level (default: ");
  print_value ("%d", default_verbosity_level);
  print_info (")\n");
}

bool
loadCloud (const std::string &filename, Cloud &cloud)
{
  TicToc tt;
  print_highlight ("Loading ");
  print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
  print_info ("Available dimensions: ");
  print_value ("%s\n", getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const Cloud &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving ");
  print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", output.width * output.height);
  print_info (" points]\n");
}

/*
void
getInitialParams (ConstCloudPtr &original, ConstCloudPtr &input)
{
  // Normal estimation*
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
  tree->setInputCloud (input);
  n.setInputCloud (input);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  concatenateFields (*input, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (100.0);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle (M_PI/4); // 45 degrees
  gp3.setMinimumAngle (M_PI/18); // 10 degrees
  gp3.setMaximumAngle (2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency (false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // get the polygonmesh cloud
  CloudPtr tri_cloud (new Cloud);
  fromPCLPointCloud2 (triangles.cloud, *tri_cloud);

  accumulator_set<float, stats<tag::median(with_p_square_quantile) > > dist_acc, angle_acc;

  for (int t = 0; t < triangles.polygons.size (); ++t)
  {
    // cropping input cloud to only those points within the first triangle
    CropHull<PointXYZ> ch;
    ch.setInputCloud (original);
    ch.setHullCloud (tri_cloud);
    ch.setDim (2);
    std::vector<Vertices> first_triangle;
    first_triangle.push_back (triangles.polygons[t]);
    ch.setHullIndices (first_triangle);
    std::vector<int> hidx;
    ch.filter (hidx);

    // getting vertices of first triangle
    PointXYZ a = tri_cloud->points[triangles.polygons[t].vertices[0]];
    PointXYZ b = tri_cloud->points[triangles.polygons[t].vertices[1]];
    PointXYZ c = tri_cloud->points[triangles.polygons[t].vertices[2]];

    // get plane defined by vertices
    Eigen::Hyperplane<float, 3> eigen_plane =
      Eigen::Hyperplane<float, 3>::Through (a.getArray3fMap (),
                                            b.getArray3fMap (),
                                            c.getArray3fMap ());

    Eigen::Vector4f pn;
    pn[0] = eigen_plane.normal ()[0];
    pn[1] = eigen_plane.normal ()[1];
    pn[2] = eigen_plane.normal ()[2];
    pn[3] = 0.0f;

    Eigen::Vector4f aa = a.getArray4fMap ();
    Eigen::Vector4f bb = b.getArray4fMap ();
    Eigen::Vector4f cc = c.getArray4fMap ();

    float m_pi_over_two = M_PI * 0.5f;

    for (int i = 0; i < hidx.size (); ++i)
    {
      Eigen::Vector3f angles;
      Eigen::Vector3f p3 = original->points[hidx[i]].getArray3fMap ();
      Eigen::Vector4f p4 = original->points[hidx[i]].getArray4fMap ();
      angles[0] = m_pi_over_two - getAngle3D (pn, p4-aa);
      angles[1] = m_pi_over_two - getAngle3D (pn, p4-bb);
      angles[2] = m_pi_over_two - getAngle3D (pn, p4-cc);
      float dist = eigen_plane.absDistance (p3);

      // push distance and angle
      dist_acc(dist);
      angle_acc(angles[0]);
      angle_acc(angles[1]);
      angle_acc(angles[2]);
    }
  }
  std::cerr << median(dist_acc) << ", " << median(angle_acc)*180/M_PI << std::endl;
}
*/

void
iterate (ConstCloudPtr &original, ConstCloudPtr &input, Cloud &output, float max_dist_thresh, float max_angle_thresh, bool adapt=false)
{
  // Normal estimation*
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
  tree->setInputCloud (input);
  n.setInputCloud (input);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  concatenateFields (*input, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (100.0);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle (M_PI/4); // 45 degrees
  gp3.setMinimumAngle (M_PI/18); // 10 degrees
  gp3.setMaximumAngle (2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency (false);

  // Get result
  std::cerr << "Points into triangulation " << cloud_with_normals->points.size () << std::endl;
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // get the polygonmesh cloud
  CloudPtr tri_cloud (new Cloud);
  fromPCLPointCloud2 (triangles.cloud, *tri_cloud);

  float m_pi_over_two = M_PI * 0.5f;

  std::cerr << "Triangulation composed of " << triangles.polygons.size () << " triangles" << std::endl;

  ba::accumulator_set<float, ba::stats<ba::tag::median (ba::with_p_square_quantile), ba::tag::min, ba::tag::max > > dist_acc, angle_acc, edge_acc;

  for (int t = 0; t < triangles.polygons.size (); ++t)
  {
    // cropping input cloud to only those points within the first triangle
    CropHull<PointXYZ> ch;
    ch.setInputCloud (original);
    ch.setHullCloud (tri_cloud);
    ch.setDim (2);
    std::vector<Vertices> first_triangle;
    first_triangle.push_back (triangles.polygons[t]);
    ch.setHullIndices (first_triangle);
    std::vector<int> hidx;
    ch.filter (hidx);

    // getting vertices of first triangle
    PointXYZ a = tri_cloud->points[triangles.polygons[t].vertices[0]];
    PointXYZ b = tri_cloud->points[triangles.polygons[t].vertices[1]];
    PointXYZ c = tri_cloud->points[triangles.polygons[t].vertices[2]];

    edge_acc(euclideanDistance(a,b)); 
    edge_acc(euclideanDistance(b,c)); 
    edge_acc(euclideanDistance(c,a)); 

    // get plane defined by vertices
    Eigen::Hyperplane<float, 3> eigen_plane =
      Eigen::Hyperplane<float, 3>::Through (a.getArray3fMap (),
                                            b.getArray3fMap (),
                                            c.getArray3fMap ());

    Eigen::Vector4f pn;
    pn[0] = eigen_plane.normal ()[0];
    pn[1] = eigen_plane.normal ()[1];
    pn[2] = eigen_plane.normal ()[2];
    pn[3] = 0.0f;

    if (pn[2] < 0)
      pn *= -1;

    Eigen::Vector4f aa = a.getArray4fMap ();
    Eigen::Vector4f bb = b.getArray4fMap ();
    Eigen::Vector4f cc = c.getArray4fMap ();

    for (int i = 0; i < hidx.size (); ++i)
    {
      Eigen::Vector3f angles;
      Eigen::Vector3f p3 = original->points[hidx[i]].getArray3fMap ();
      Eigen::Vector4f p4 = original->points[hidx[i]].getArray4fMap ();
      angles[0] = m_pi_over_two - getAngle3D (pn, p4-aa);
      angles[1] = m_pi_over_two - getAngle3D (pn, p4-bb);
      angles[2] = m_pi_over_two - getAngle3D (pn, p4-cc);
      float dist = eigen_plane.absDistance (p3);

      // this happens when the current point is one of the vertices
      if (pcl_isnan (angles[0]) || pcl_isnan (angles[1]) || pcl_isnan (angles[2]))
        continue;

      // push distance and angle
      dist_acc (dist);
      angle_acc (angles[0]);
      angle_acc (angles[1]);
      angle_acc (angles[2]);
    }
  }

  float dist_thresh = max_dist_thresh;
  float angle_thresh = max_angle_thresh;

  if (adapt)
  {
    if (!pcl_isnan (ba::median (dist_acc)))
      dist_thresh = ba::median (dist_acc);

    if (!pcl_isnan (ba::median (angle_acc)))
      angle_thresh = ba::median (angle_acc);

    if (dist_thresh > max_dist_thresh) dist_thresh = max_dist_thresh;
    if (angle_thresh > max_angle_thresh) angle_thresh = max_angle_thresh;
  }

  //std::cerr << "Distance threshold set at " << dist_thresh << " (median was " << median (dist_acc) << ")" << std::endl;
  printf("Distance threshold at %.2f (%.2f, %.2f, %.2f)\n", dist_thresh, (ba::min)(dist_acc), ba::median(dist_acc), (ba::max)(dist_acc));
  //std::cerr << "Angle threshold set at " << rad2deg (angle_thresh) << " (median was " << rad2deg (median (angle_acc)) << ")" << std::endl;
  printf("Angle threshold at %.2f (%.2f, %.2f, %.2f)\n", rad2deg(angle_thresh), rad2deg((ba::min)(angle_acc)), rad2deg(ba::median(angle_acc)), rad2deg((ba::max)(angle_acc)));
  printf("Edge lengths (%.2f, %.2f, %.2f)\n", (ba::min)(edge_acc), ba::median(edge_acc), (ba::max)(edge_acc));



  PointIndicesPtr addtoground (new PointIndices);

  for (int t = 0; t < triangles.polygons.size (); ++t)
  {
    // cropping input cloud to only those points within the first triangle
    CropHull<PointXYZ> ch;
    ch.setInputCloud (original);
    ch.setHullCloud (tri_cloud);
    ch.setDim (2);
    std::vector<Vertices> first_triangle;
    first_triangle.push_back (triangles.polygons[t]);
    ch.setHullIndices (first_triangle);
    std::vector<int> hidx;
    ch.filter (hidx);

    // getting vertices of first triangle
    PointXYZ a = tri_cloud->points[triangles.polygons[t].vertices[0]];
    PointXYZ b = tri_cloud->points[triangles.polygons[t].vertices[1]];
    PointXYZ c = tri_cloud->points[triangles.polygons[t].vertices[2]];

    // get plane defined by vertices
    Eigen::Hyperplane<float, 3> eigen_plane =
      Eigen::Hyperplane<float, 3>::Through (a.getArray3fMap (),
                                            b.getArray3fMap (),
                                            c.getArray3fMap ());

    Eigen::Vector4f pn;
    pn[0] = eigen_plane.normal ()[0];
    pn[1] = eigen_plane.normal ()[1];
    pn[2] = eigen_plane.normal ()[2];
    pn[3] = 0.0f;

    if (pn[2] < 0)
      pn *= -1;

    Eigen::Vector4f aa = a.getArray4fMap ();
    Eigen::Vector4f bb = b.getArray4fMap ();
    Eigen::Vector4f cc = c.getArray4fMap ();

    bool newpoint = false;
    float bestdist = std::numeric_limits<float>::max ();
    float bestangle = std::numeric_limits<float>::max ();
    int bestidx = 0;
    for (int i = 0; i < hidx.size (); ++i)
    {
      Eigen::Vector3f angles;
      Eigen::Vector3f p3 = original->points[hidx[i]].getArray3fMap ();
      Eigen::Vector4f p4 = original->points[hidx[i]].getArray4fMap ();
      angles[0] = m_pi_over_two - getAngle3D (pn, p4-aa);
      angles[1] = m_pi_over_two - getAngle3D (pn, p4-bb);
      angles[2] = m_pi_over_two - getAngle3D (pn, p4-cc);
      float dist = eigen_plane.absDistance (p3);

      // this happens when the current point is one of the vertices
      if (pcl_isnan (angles[0]) || pcl_isnan (angles[1]) || pcl_isnan (angles[2]))
        continue;

      // these should be identical, but they aren't in practice, but why
      //if (dist < dist_thresh && angles[0] < angle_thresh && angles[1] < angle_thresh && angles[2] < angle_thresh)
      if (dist < dist_thresh && angles.maxCoeff () < angle_thresh)
      {
        addtoground->indices.push_back (hidx[i]);
        /*
        if (dist < bestdist && angles.maxCoeff() < bestangle)
        {
          newpoint = true;
          bestdist = dist;
          bestangle = angles.maxCoeff();
          bestidx = hidx[i];
        }
        */
      }
      else
      {
        /*
        // check mirror point
        float da = (p4-aa).norm ();
        float db = (p4-bb).norm ();
        float dc = (p4-cc).norm ();
        if (da < db && da < dc)
        {
          // closest to vertex a
          // find the mirror point
          // find the triangle it belongs to
          // find that triangle's normal
          // compute distance to triangle
          // compute angles to vertices
        }
        else if (db < da && db < dc)
        {
        }
        else if (dc < da && dc < db)
        {
        }
        */
      }
    }
//    if (newpoint)
//      addtoground->indices.push_back (bestidx);
  }

  ExtractIndices<PointXYZ> extract;
  CloudPtr ground (new Cloud);
  extract.setInputCloud (original);
  extract.setIndices (addtoground);
  extract.filter (output);
  output += *input;
}

void
compute (ConstCloudPtr &input, Cloud &output, float resolution, float dist_thresh, float angle_thresh, int max_iters)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing \n");

  // start by finding grid minimums (user variable res, larger than buildings)
  CloudPtr cloud_mins (new Cloud);
  GridMinimum<PointXYZ> gm (resolution);
  gm.setInputCloud (input);
  gm.filter (*cloud_mins);

  CloudPtr cloud (new Cloud);
  CloudPtr cloud_f (new Cloud);
  cloud = cloud_mins;
  for (int i = 0; i < max_iters; ++i)
  {
    std::cerr << "Densification starts with " << cloud->points.size () << " out of " << input->points.size () << " points." << std::endl;
    if (i == 0)
      iterate (input, cloud, *cloud_f, dist_thresh, 40*M_PI/180, false);
    else
      iterate (input, cloud, *cloud_f, dist_thresh, 6*M_PI/180, true);
    int new_pts = cloud_f->points.size () - cloud->points.size ();

    std::cerr << "Iteration " << i << " added " << new_pts << " points." << std::endl;
    std::cerr << "Ground now has " << cloud_f->points.size () << " points." << std::endl;

    cloud.swap (cloud_f);
    if (new_pts == 0)
      break;
  }
  output = *cloud;

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", output.width * output.height);
  print_info (" points]\n");
}

int
batchProcess (const vector<string> &pcd_files, string &output_dir, float resolution, float dist_thresh, float angle_thresh, int max_iters)
{
  vector<string> st;
  for (size_t i = 0; i < pcd_files.size (); ++i)
  {
    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (pcd_files[i], *cloud))
      return (-1);

    // Perform the feature estimation
    Cloud output;
    compute (cloud, output, resolution, dist_thresh, angle_thresh, max_iters);

    // Prepare output file name
    string filename = pcd_files[i];
    boost::trim (filename);
    boost::split (st, filename, boost::is_any_of ("/\\"), boost::token_compress_on);

    // Save into the second file
    stringstream ss;
    ss << output_dir << "/" << st.at (st.size () - 1);
    saveCloud (ss.str (), output);
  }
  return (0);
}


/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Filter a point cloud using the pcl::ProgressiveDensification. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  float resolution = default_resolution;
  float dist_thresh = default_dist_thresh;
  float angle_thresh = default_angle_thresh;
  int verbosity_level = default_verbosity_level;
  int max_iters = default_max_iters;
  parse_argument (argc, argv, "-resolution", resolution);
  parse_argument (argc, argv, "-dist_thresh", dist_thresh);
  parse_argument (argc, argv, "-angle_thresh", angle_thresh);

  // convert angle from deg to rad
  angle_thresh = deg2rad (angle_thresh);

  parse_argument (argc, argv, "-verbosity", verbosity_level);
  parse_argument (argc, argv, "-max_iters", max_iters);
  string input_dir, output_dir;
  if (parse_argument (argc, argv, "-input_dir", input_dir) != -1)
  {
    PCL_INFO ("Input directory given as %s. Batch process mode on.\n", input_dir.c_str ());
    if (parse_argument (argc, argv, "-output_dir", output_dir) == -1)
    {
      PCL_ERROR ("Need an output directory! Please use -output_dir to continue.\n");
      return (-1);
    }

    // Both input dir and output dir given, switch into batch processing mode
    batch_mode = true;
  }

  switch (verbosity_level)
  {
    case 0:
      setVerbosityLevel (L_ALWAYS);
      break;

    case 1:
      setVerbosityLevel (L_ERROR);
      break;

    case 2:
      setVerbosityLevel (L_WARN);
      break;

    case 3:
      setVerbosityLevel (L_INFO);
      break;

    case 4:
      setVerbosityLevel (L_DEBUG);
      break;

    default:
      setVerbosityLevel (L_VERBOSE);
      break;
  }

  if (!batch_mode)
  {
    // Parse the command line arguments for .pcd files
    std::vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (argv[p_file_indices[0]], *cloud))
      return (-1);

    // Perform the feature estimation
    Cloud output;
    compute (cloud, output, resolution, dist_thresh, angle_thresh, max_iters);

    // Save into the second file
    saveCloud (argv[p_file_indices[1]], output);
  }
  else
  {
    if (input_dir != "" && boost::filesystem::exists (input_dir))
    {
      vector<string> pcd_files;
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (input_dir); itr != end_itr; ++itr)
      {
        // Only add PCD files
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD")
        {
          pcd_files.push_back (itr->path ().string ());
          PCL_INFO ("[Batch processing mode] Added %s for processing.\n", itr->path ().string ().c_str ());
        }
      }
      batchProcess (pcd_files, output_dir, resolution, dist_thresh, angle_thresh, max_iters);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}

