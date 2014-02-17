/*
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

#ifndef PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_
#define PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/grid_minimum.h>

struct cloud_point_index_idx 
{
  unsigned int idx;
  unsigned int cloud_point_index;

  cloud_point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
  bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::GridMinimum<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  std::vector<int> indices;

  output.is_dense = true;
  applyFilterIndices (indices);
  pcl::copyPointCloud<PointT> (*input_, indices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::GridMinimum<PointT>::applyFilterIndices (std::vector<int> &indices)
{
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

  PCL_DEBUG( "raw coordinate min/max\n" );
  PCL_DEBUG( "%f %f\n", min_p[0], max_p[0] );
  PCL_DEBUG( "%f %f\n", min_p[1], max_p[1] );
  PCL_DEBUG( "\n" );

  // Check that the leaf size is not too small, given the size of the data
  int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;

  PCL_DEBUG( "dx = %d, dy = %d\n", dx, dy );
  PCL_DEBUG( "\n" );

  if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));

  PCL_DEBUG( "Min/max bounding box values\n" );
  PCL_DEBUG( "%d %d\n", min_b_[0], max_b_[0] );
  PCL_DEBUG( "%d %d\n", min_b_[1], max_b_[1] );
  PCL_DEBUG( "\n" );

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  PCL_DEBUG( "number of divisions %d, %d\n", div_b_[0], div_b_[1] );
  PCL_DEBUG( "\n" );

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], 0, 0);

  PCL_DEBUG( "multipliers %d, %d\n", divb_mul_[0], divb_mul_[1] );
  PCL_DEBUG( "\n" );

  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve (indices_->size ());

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[*it].x) || 
            !pcl_isfinite (input_->points[*it].y) || 
            !pcl_isfinite (input_->points[*it].z))
          continue;

      int ijk0 = static_cast<int> (floor (input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
      int ijk1 = static_cast<int> (floor (input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1];
      index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), *it));

    }
  PCL_DEBUG( "%f %f %d %d\n", inverse_leaf_size_[0], inverse_leaf_size_[1], min_b_[0], min_b_[1]);

  PCL_DEBUG( "index vector size (i.e., number of indexed points) %d\n", index_vector.size() );

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  unsigned int total = 0;
  unsigned int index = 0;
  // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
  // index_vector belonging to the voxel which corresponds to the i-th output point,
  // and of the first point not belonging to.
  std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
  // Worst case size
  first_and_last_indices_vector.reserve (index_vector.size ());
  while (index < index_vector.size ()) 
  {
    unsigned int i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
      ++i;
    if (i - index >= min_points_per_grid_)
    {
      ++total;
      first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
    }
    index = i;
  }

  PCL_DEBUG( "number of passing voxels %d\n", total );

  // Fourth pass: compute centroids, insert them into their final position
  indices.resize (total);
  if (save_leaf_layout_)
  {
    try
    { 
      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
      uint32_t new_layout_size = div_b_[0]*div_b_[1];

      //This is the number of elements that need to be re-initialized to -1
      uint32_t reinit_size = std::min (static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
      for (uint32_t i = 0; i < reinit_size; i++)
      {
        leaf_layout_[i] = -1;
      }        
      leaf_layout_.resize (new_layout_size, -1);           
    }
    catch (std::bad_alloc&)
    {
      throw PCLException("GridMinimum bin size is too low; impossible to allocate memory for layout", 
        "grid_minimum.hpp", "applyFilter");	
    }
    catch (std::length_error&)
    {
      throw PCLException("GridMinimum bin size is too low; impossible to allocate memory for layout", 
        "grid_minimum.hpp", "applyFilter");	
    }
  }
  
  index = 0;

  for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
  {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
	unsigned int first_index = first_and_last_indices_vector[cp].first;
	unsigned int last_index = first_and_last_indices_vector[cp].second;
        unsigned int min_index;
        float min_z;
    if (!downsample_all_data_) 
    {
      min_z = input_->points[index_vector[first_index].cloud_point_index].z;
      min_index = index_vector[first_index].cloud_point_index;
      
      PCL_DEBUG( "min z starts at %f\n", min_z );
    }

    for (unsigned int i = first_index + 1; i < last_index; ++i) 
    {
      if (!downsample_all_data_) 
      {
        if (input_->points[index_vector[i].cloud_point_index].z < min_z)
        {
          min_z = input_->points[index_vector[i].cloud_point_index].z;
          min_index = index_vector[i].cloud_point_index;
          PCL_DEBUG( "min z is now %f\n", min_z );
        }
      }
    }


    // index is centroid final position in resulting PointCloud
    if (save_leaf_layout_)
      leaf_layout_[index_vector[first_index].idx] = index;


    // store centroid
    // Do we need to process all the fields?
    if (!downsample_all_data_) 
    {
      indices[index] = min_index;
      PCL_DEBUG( "final min z is %f\n", min_z );
      PCL_DEBUG( "indexed point is %f, %f, %f\n", input_->points[indices[index]].x, input_->points[indices[index]].y, input_->points[indices[index]].z );
    }
    ++index;
  }

  oii = indices.size();

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_GridMinimum(T) template class PCL_EXPORTS pcl::GridMinimum<T>;

#endif    // PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_

