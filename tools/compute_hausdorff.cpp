/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2014, Bradley J Chambers
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
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

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
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
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
compute (Cloud &cloudA, Cloud &cloudB)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  // compare A to B
  pcl::search::KdTree<PointType> treeB;
  treeB.setInputCloud( cloudB.makeShared() );
  float max_dist_A = -std::numeric_limits<float>::max();
  for (int i = 0; i < cloudA.points.size(); ++i)
  {
    std::vector<int> indices( 1 );
    std::vector<float> sqr_distances( 1 );

    treeB.nearestKSearch( cloudA.points[i], 1, indices, sqr_distances );
    if (sqr_distances[0] > max_dist_A)
      max_dist_A = sqr_distances[0];
  }

  // compare B to A
  pcl::search::KdTree<PointType> treeA;
  treeA.setInputCloud( cloudA.makeShared() );
  float max_dist_B = -std::numeric_limits<float>::max();
  for (int i = 0; i < cloudB.points.size(); ++i)
  {
    std::vector<int> indices( 1 );
    std::vector<float> sqr_distances( 1 );

    treeA.nearestKSearch( cloudB.points[i], 1, indices, sqr_distances );
    if (sqr_distances[0] > max_dist_B)
      max_dist_B = sqr_distances[0];
  }

  float dist = std::max(max_dist_A, max_dist_B);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%F, %f, %f", max_dist_A, max_dist_B, dist); print_info (" ]\n");
}

/*
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
*/

/*
int
//batchProcess (const vector<string> &pcd_files, string &output_dir, float resolution)
batchProcess (const vector<string> &pcd_files, string &output_dir)
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
    //compute (*cloud, output, resolution);
    compute (*cloud, output);

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
*/

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute Hausdorff distance between point clouds. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

/*
  // Command line parsing
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
*/

  if (!batch_mode)
  {
    // Parse the command line arguments for .pcd files
    std::vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need at least two PCD files to compute distance.\n");
      return (-1);
    }

    // Load the first file
    Cloud::Ptr cloudA (new Cloud);
    if (!loadCloud (argv[p_file_indices[0]], *cloudA))
      return (-1);

    // Load the second file
    Cloud::Ptr cloudB (new Cloud);
    if (!loadCloud (argv[p_file_indices[1]], *cloudB))
      return (-1);

    // Compute the Hausdorff distance
    compute (*cloudA, *cloudB);
  }
/*
  else
  {
    if (input_dir != "" && boost::filesystem::exists (input_dir))
    {
      vector<string> pcd_files;
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (input_dir); itr != end_itr; ++itr)
      {
        // Only add PCD files
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
        {
          pcd_files.push_back (itr->path ().string ());
          PCL_INFO ("[Batch processing mode] Added %s for processing.\n", itr->path ().string ().c_str ());
        }
      }
      //batchProcess (pcd_files, output_dir, resolution);
      batchProcess (pcd_files, output_dir);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
*/
}
