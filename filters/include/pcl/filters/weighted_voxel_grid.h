/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_WEIGHTED_VOXEL_GRID_COVARIANCE_H_
#define PCL_WEIGHTED_VOXEL_GRID_COVARIANCE_H_

#include <pcl/filters/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  /** \brief A brief description.
    * \author Bradley J Chambers
    */
  template<typename PointT>
  class WeightedVoxelGrid : public VoxelGrid<PointT>
  {
    protected:
      using VoxelGrid<PointT>::input_;
      using Filter<PointT>::getClassName;
      using VoxelGrid<PointT>::inverse_leaf_size_;
      using VoxelGrid<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using VoxelGrid<PointT>::leaf_size_;
      using VoxelGrid<PointT>::leaf_layout_;
      using VoxelGrid<PointT>::filter_field_name_;
      using VoxelGrid<PointT>::filter_limit_min_;
      using VoxelGrid<PointT>::filter_limit_max_;
      using VoxelGrid<PointT>::filter_limit_negative_;
      using VoxelGrid<PointT>::max_b_;
      using VoxelGrid<PointT>::min_b_;
      using VoxelGrid<PointT>::div_b_;
      using VoxelGrid<PointT>::divb_mul_;
      using VoxelGrid<PointT>::downsample_all_data_;
      using VoxelGrid<PointT>::save_leaf_layout_;
        
      typedef typename pcl::traits::fieldList<PointT>::type FieldList;
      typedef typename Filter<PointT>::PointCloud PointCloud;
      typedef boost::shared_ptr< VoxelGrid<PointT> > Ptr;
      typedef boost::shared_ptr< const VoxelGrid<PointT> > ConstPtr;

    public:

      /** \brief Empty constructor. */
      WeightedVoxelGrid ()
      {
        //weights_ = new std::vector<float>();
        leaf_size_ = Eigen::Vector4f::Zero ();
        inverse_leaf_size_ = Eigen::Array4f::Zero ();
        downsample_all_data_ = true;
        save_leaf_layout_ = false;
        //leaf_layout_ = NULL;
        min_b_ = Eigen::Vector4i::Zero ();
        max_b_ = Eigen::Vector4i::Zero ();
        div_b_ = Eigen::Vector4i::Zero ();
        divb_mul_ = Eigen::Vector4i::Zero ();
        filter_field_name_ = "";
        filter_limit_min_ = -FLT_MAX;
        filter_limit_max_ = FLT_MAX;
        filter_limit_negative_ = false;
        filter_name_ = "WeightedVoxelGrid";
      }
    
    inline void
    setWeights( std::vector<float> &weights )
    {
      weights_ = weights;
    }

    protected:

      /** \brief Filter cloud and initializes voxel structure.
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       */
      void applyFilter (PointCloud &output);
    
      std::vector<float> weights_;
      };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/weighted_voxel_grid.hpp>
#endif

#endif  //#ifndef PCL_WEIGHTED_VOXEL_GRID_H_
