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
 */

#ifndef PCL_SEGMENTATION_PROGRESSIVE_DENSIFICATION_FILTER_HPP_
#define PCL_SEGMENTATION_PROGRESSIVE_DENSIFICATION_FILTER_HPP_

#include <pcl/segmentation/progressive_densification_filter.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include <Eigen/Geometry>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

namespace ba = boost::accumulators;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveDensificationFilter<PointT>::ProgressiveDensificationFilter () :
  resolution_ (15.0f),
  dist_thresh_ (1.5f),
  angle_thresh_ (6.0f),
  max_iters_ (10)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveDensificationFilter<PointT>::~ProgressiveDensificationFilter ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveDensificationFilter<PointT>::densify (const typename pcl::PointCloud<PointT>::ConstPtr &original, std::vector<int> &ground, float max_dist_thresh, float max_angle_thresh, bool adapt)
{
  typename pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
  pcl::copyPointCloud<PointT> (*original, ground, *input);

  // Normal estimation*
  pcl::NormalEstimation<PointT, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (input);
  n.setInputCloud (input);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  concatenateFields (*input, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  search::KdTree<pcl::PointNormal>::Ptr tree2 (new search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<pcl::PointNormal> gp3;
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
  typename pcl::PointCloud<PointT>::Ptr tri_cloud (new pcl::PointCloud<PointT>);
  fromPCLPointCloud2 (triangles.cloud, *tri_cloud);

  float m_pi_over_two = M_PI * 0.5f;

  ba::accumulator_set<float, ba::stats<ba::tag::median (ba::with_p_square_quantile), ba::tag::min, ba::tag::max > > dist_acc, angle_acc, edge_acc;

  for (int t = 0; t < triangles.polygons.size (); ++t)
  {
    // cropping input cloud to only those points within the first triangle
    CropHull<PointT> ch;
    ch.setInputCloud (original);
    ch.setHullCloud (tri_cloud);
    ch.setDim (2);
    std::vector<Vertices> first_triangle;
    first_triangle.push_back (triangles.polygons[t]);
    ch.setHullIndices (first_triangle);
    std::vector<int> hidx;
    ch.filter (hidx);

    // getting vertices of first triangle
    PointT a = tri_cloud->points[triangles.polygons[t].vertices[0]];
    PointT b = tri_cloud->points[triangles.polygons[t].vertices[1]];
    PointT c = tri_cloud->points[triangles.polygons[t].vertices[2]];

    edge_acc (euclideanDistance (a,b));
    edge_acc (euclideanDistance (b,c));
    edge_acc (euclideanDistance (c,a));

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

  PCL_DEBUG ("Distance threshold: %.2f (min/median/max = %.2f/%.2f/%.2f)\n", dist_thresh, (ba::min) (dist_acc), ba::median (dist_acc), (ba::max) (dist_acc));
  PCL_DEBUG ("Angle threshold: %.2f (min/median/max = %.2f/%.2f/%.2f)\n", rad2deg (angle_thresh), rad2deg ((ba::min) (angle_acc)), rad2deg (ba::median (angle_acc)), rad2deg ((ba::max) (angle_acc)));
  PCL_DEBUG ("Edge lengths (min/median/max = %.2f/%.2f/%.2f)\n", (ba::min) (edge_acc), ba::median (edge_acc), (ba::max) (edge_acc));

  PointIndicesPtr addtoground (new PointIndices);
  addtoground->indices = ground;

  for (int t = 0; t < triangles.polygons.size (); ++t)
  {
    // cropping input cloud to only those points within the first triangle
    CropHull<PointT> ch;
    ch.setInputCloud (original);
    ch.setHullCloud (tri_cloud);
    ch.setDim (2);
    std::vector<Vertices> first_triangle;
    first_triangle.push_back (triangles.polygons[t]);
    ch.setHullIndices (first_triangle);
    std::vector<int> hidx;
    ch.filter (hidx);

    // getting vertices of first triangle
    PointT a = tri_cloud->points[triangles.polygons[t].vertices[0]];
    PointT b = tri_cloud->points[triangles.polygons[t].vertices[1]];
    PointT c = tri_cloud->points[triangles.polygons[t].vertices[2]];

    Eigen::Vector3f edges;
    edges[0] = euclideanDistance (a,b);
    edges[1] = euclideanDistance (b,c);
    edges[2] = euclideanDistance (c,a);
    float weight = edges.maxCoeff () / 5.0f;
    if (weight > 1.0f)
      weight = 1.0f;
    else
      weight *= weight;
    // find max dist ab, ac, bc
    // angle weight = max dist / 5.0
    // if weight < 1, weight *= weight
    // angle_thresh *= weight

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

//    bool newpoint = false;
//    float bestdist = std::numeric_limits<float>::max ();
//    float bestangle = std::numeric_limits<float>::max ();
//    int bestidx = 0;
    int potential_mirror_pts = 0;
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
      if (dist < dist_thresh && angles.maxCoeff () < (angle_thresh*weight))
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
        potential_mirror_pts++;
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
    PCL_DEBUG ("Triangle %d has %d potential mirror points.\n", t, potential_mirror_pts);
//    if (newpoint)
//      addtoground->indices.push_back (bestidx);
  }

  ground = addtoground->indices;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveDensificationFilter<PointT>::extract (std::vector<int>& ground)
{
  bool segmentation_is_possible = initCompute ();
  if (!segmentation_is_possible)
  {
    deinitCompute ();
    return;
  }

  // Ground indices are initially limited to those points in the input cloud we
  // wish to process
  ground = *indices_;

  // Limit filtering to those points currently considered ground returns
  typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
  pcl::copyPointCloud<PointT> (*input_, ground, *cloud_in);

  // start by finding grid minimums (user variable res, larger than buildings)
  typename pcl::PointCloud<PointT>::Ptr cloud_mins (new pcl::PointCloud<PointT>);
  GridMinimum<PointT> gm (resolution_);
  gm.setInputCloud (cloud_in);
  gm.filter (ground);

  for (int i = 0; i < max_iters_; ++i)
  {
    int prev_points = ground.size ();
    PCL_DEBUG ("Densification starts with %d out of %d points labeled as ground (%.2f%%).\n", prev_points, cloud_in->points.size (), 100.0f*static_cast<float> (prev_points)/static_cast<float> (cloud_in->points.size ()));
    if (i == 0)
      densify (cloud_in, ground, dist_thresh_, angle_thresh_, false); // should this be larger for the first iteration, e.g., 40deg?
    else
      densify (cloud_in, ground, dist_thresh_, angle_thresh_, true);
    int new_pts = ground.size () - prev_points;

    PCL_DEBUG ("Iteration %d added %d points.\n", i+1, new_pts);

    if (new_pts == 0)
      break;
  }

  deinitCompute ();
}

#define PCL_INSTANTIATE_ProgressiveDensificationFilter(T) template class pcl::ProgressiveDensificationFilter<T>;

#endif    // PCL_SEGMENTATION_PROGRESSIVE_DENSIFICATION_FILTER_HPP_

