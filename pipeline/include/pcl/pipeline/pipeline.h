/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2013, Willow Garage, Inc.
 *  Copyright (c) 2013-, RadiantBlue Technologies, Inc.
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
 *
 */

#ifndef PCL_FILTERS_PIPELINE_H_
#define PCL_FILTERS_PIPELINE_H_

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief @b Pipeline composes a linear series of PCL operations to be applied to the source point cloud.
    * \details TBD
    * <br><br>
    * Usage example:
    * \code
    * pcl::Pipeline<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    * ptfilter.setInputCloud (cloud_in);
    * ptfilter.setJSON (filename);
    * ptfilter.filter (*indices_x);
    * \endcode
    * \author Bradley J Chambers
    * \ingroup pipeline
    */
  template <typename PointT>
  class Pipeline : public FilterIndices<PointT>
  {
    protected:
      typedef typename FilterIndices<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

    public:

      typedef boost::shared_ptr< Pipeline<PointT> > Ptr;
      typedef boost::shared_ptr< const Pipeline<PointT> > ConstPtr;


      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      Pipeline (bool extract_removed_indices = false) :
        FilterIndices<PointT>::FilterIndices (extract_removed_indices)
      {
        filter_name_ = "Pipeline";
      }

      /** \brief Provide the name of the JSON file describing the PCL pipeline.
        * \param[in] filename the name of the JSON file describing the PCL pipeline
        */
      inline void
      setJSON (const std::string &filename)
      {
        filename_ = filename;
      }


    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are stored in a separate point cloud.
        * \param[out] output The resultant point cloud.
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (std::vector<int> &indices)
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (std::vector<int> &indices);

    private:
      /** \brief The name of the JSON file describing the PCL pipeline. */
      std::string filename_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Pipeline uses the base Filter class methods to compose a linear series of operations.
    * \author Bradley J Chambers
    * \ingroup pipeline
    */
  template<>
  class PCL_EXPORTS Pipeline<pcl::PCLPointCloud2> : public Filter<pcl::PCLPointCloud2>
  {
    typedef pcl::PCLPointCloud2 PCLPointCloud2;
    typedef PCLPointCloud2::Ptr PCLPointCloud2Ptr;
    typedef PCLPointCloud2::ConstPtr PCLPointCloud2ConstPtr;

    using Filter<pcl::PCLPointCloud2>::removed_indices_;
    using Filter<pcl::PCLPointCloud2>::extract_removed_indices_;

    public:
      /** \brief Constructor. */
      Pipeline (bool extract_removed_indices = false) :
        Filter<pcl::PCLPointCloud2>::Filter (extract_removed_indices),
	filename_ ("")
      {
        filter_name_ = "Pipeline";
      }

      /** \brief Provide the name of the JSON file describing the PCL pipeline.
        * \param[in] filename the name of the JSON file describing the PCL pipeline
        */
      inline void
      setJSON (const std::string &filename)
      {
        filename_ = filename;
      }

    protected:
      void
      applyFilter (PCLPointCloud2 &output);

    private:
      /** \brief The name of the JSON file describing the PCL pipeline. */
      std::string filename_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/pipeline.hpp>
#endif

#endif  // PCL_FILTERS_PIPELINE_H_

