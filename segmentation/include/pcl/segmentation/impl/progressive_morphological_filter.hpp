/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013, Bradley J Chambers
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

#ifndef PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
#define PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::ProgressiveMorphologicalFilter () :
  max_window_size_ (33),
  slope_ (0.7f),
  max_distance_ (10.0f),
  initial_distance_ (0.15f),
  ground_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::~ProgressiveMorphologicalFilter ()
{
  ground_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::ProgressiveMorphologicalFilter<PointT>::getMaxWindowSize ()
{
  return (max_window_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setMaxWindowSize (int max_window_size)
{
  max_window_size_ = max_window_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getSlope () const
{
  return (slope_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setSlope (float slope)
{
  slope_ = slope;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getMaxDistance () const
{
  return (max_distance_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setMaxDistance (float max_distance)
{
  max_distance_ = max_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getInitialDistance () const
{
  return (initial_distance_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setInitialDistance (float initial_distance)
{
  initial_distance_ = initial_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::extract (std::vector <int>& ground)
{
  ground_.clear ();
  ground.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_out( new pcl::PointCloud<PointT> );

  float window_size = 3.0f;
  int level = 0;

  while (window_size < max_window_size_)
  {
    pcl::morphologicalOpen<PointT>(*input_, window_size, *cloud_out);

    //compute threshold
    //passthrough for only points below threshold

    level++;
    window_size = static_cast<float>(2 * std::pow(2, level) + 1);
  }

  deinitCompute ();
}

#define PCL_INSTANTIATE_ProgressiveMorphologicalFilter(T) template class pcl::ProgressiveMorphologicalFilter<T>;

#endif    // PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
