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

#ifndef PCL_PROGRESSIVE_DENSIFICATION_FILTER_H_
#define PCL_PROGRESSIVE_DENSIFICATION_FILTER_H_

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief
    * Implements the Progressive Densification Filter for segmentation of ground points.
    * Description can be found in the article
    * "DEM generation from laser scanner data using adaptive TIN models" by P.
    * Axelsson.
    */
  template <typename PointT>
  class PCL_EXPORTS ProgressiveDensificationFilter : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::PointCloud <PointT> PointCloud;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      ProgressiveDensificationFilter ();

      virtual
      ~ProgressiveDensificationFilter ();

      /** \brief Get the resolution to be used in filtering ground returns. */
      inline float
      getResolution () const { return (resolution_); }

      /** \brief Set the resolution to be used in filtering ground returns. */
      inline void
      setResolution (float resolution) { resolution_ = resolution; }

      /** \brief Get the distance to TIN facet threshold. */
      inline float
      getDistThresh () const { return (dist_thresh_); }

      /** \brief Set the distance to TIN facet threshold. */
      inline void
      setDistThresh (float dist_thresh) { dist_thresh_ = dist_thresh; }

      /** \brief Get the angle to vertex threshold. */
      inline float
      getAngleThresh () const { return (angle_thresh_); }
      
      /** \brief Set the angle to vertex threshold. */
      inline void
      setAngleThresh (float angle_thresh) { angle_thresh_ = angle_thresh; }

      /** \brief Get the maximum number of iterations. */
      inline int
      getMaxIterations () const { return (max_iters_); }

      /** \brief Set the maximum number of iterations. */
      inline void
      setMaxIterations (int max_iters) { max_iters_ = max_iters; }

      /** \brief This method launches the segmentation algorithm and returns indices of
        * points determined to be ground returns.
        * \param[out] ground indices of points determined to be ground returns.
        */
      virtual void
      extract (std::vector<int>& ground);

    protected:

      /** \brief Resolution to be used in filtering ground returns. */
      float resolution_;

      /** \brief Distance to TIN facet threshold. */
      float dist_thresh_;

      /** \brief Angle to vertex threshold. */
      float angle_thresh_;

      /** \brief Maximum number of iterations. */
      int max_iters_;

    private:
      virtual void
      densify (const typename pcl::PointCloud<PointT>::ConstPtr &original, std::vector<int> &ground, float max_dist_thresh, float max_angle_thresh, bool adapt=false);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/progressive_densification_filter.hpp>
#endif

#endif

