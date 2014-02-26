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
 *
 */

#ifndef PCL_FILTERS_IMPL_PIPELINE_HPP_
#define PCL_FILTERS_IMPL_PIPELINE_HPP_

#include <exception>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pipeline/pipeline.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyFilter (PointCloud &output)
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
pcl::Pipeline<PointT>::applyFilterIndices (std::vector<int> &indices)
{
  // The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  try
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(filename_.c_str(), pt);

    PCL_DEBUG("\n");
    PCL_DEBUG("Processing %s\n", filename_.c_str());
    PCL_INFO("\n");
    PCL_INFO("--------------------------------------------------------------------------------\n");
    PCL_INFO("NAME:   %s (%s)\n", pt.get<std::string>("pipeline.name","").c_str(), pt.get<std::string>("pipeline.version","").c_str());
    PCL_DEBUG("HELP:   %s\n", pt.get<std::string>("pipeline.help","").c_str());
    PCL_DEBUG("AUTHOR: %s\n", pt.get<std::string>("pipeline.author","").c_str());
    PCL_INFO("--------------------------------------------------------------------------------\n");

    int step = 1;

    indices = *indices_;

    BOOST_FOREACH(boost::property_tree::ptree::value_type &vt, pt.get_child("pipeline.filters"))
    {
      //IndicesPtr temp_indices ( new std::vector<int>(indices) );
      std::vector<int> local_indices;
      typename pcl::PointCloud<PointT>::Ptr local_cloud ( new pcl::PointCloud<PointT>);
      pcl::copyPointCloud<PointT>( *input_, indices, *local_cloud );

      PCL_INFO("%d points copied\n",local_cloud->points.size());

      std::string name = vt.second.get<std::string>("name","");
      std::string help = vt.second.get<std::string>("help","");

      PCL_INFO("\n");
      PCL_INFO("   Step %d) %s\n", step++, name.c_str());
      PCL_DEBUG("      %s\n", help.c_str());

      if (name == "PassThrough")
      {
	// initial setup
	pcl::PassThrough<PointT> pass;
        pass.setInputCloud(local_cloud);
        //pass.setIndices(temp_indices);

	// parse params
        std::string field = vt.second.get<std::string>("setFilterFieldName");
	float m1 = vt.second.get<float>("setFilterLimits.min", -std::numeric_limits<float>::max());
	float m2 = vt.second.get<float>("setFilterLimits.max", std::numeric_limits<float>::max());
        
	// summarize settings
	PCL_DEBUG("      Field name: %s\n", field.c_str());
        PCL_DEBUG("      Limits: %f, %f\n", m1, m2);
	
	// set params and apply filter
	pass.setFilterFieldName(field);
	pass.setFilterLimits(m1, m2);
        pass.filter(local_indices);
      }
      else if (name == "StatisticalOutlierRemoval")
      {
	// initial setup
        pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(local_cloud);
	//sor.setIndices(temp_indices);

	// parse params
	int nr_k = vt.second.get<int>("setMeanK", 2);
	double stddev_mult = vt.second.get<double>("setStddevMulThresh", 0.0);

	// summarize settings
	PCL_DEBUG("      %d neighbors and %f multiplier\n", nr_k, stddev_mult);

	// set params and apply filter
        sor.setMeanK(nr_k);
	sor.setStddevMulThresh(stddev_mult);
	sor.filter(local_indices);

        PCL_INFO("      %d points filtered to %d following outlier removal\n", indices.size(), local_indices.size());
      }
      else if (name == "RadiusOutlierRemoval")
      {
	// initial setup
        pcl::RadiusOutlierRemoval<PointT> ror;
	ror.setInputCloud(local_cloud);
	//ror.setIndices(temp_indices);

	// parse params
	int min_neighbors = vt.second.get<int>("setMinNeighborsInRadius", 2);
	double radius = vt.second.get<double>("setRadiusSearch", 1.0);

	// summarize settings
	PCL_DEBUG("      %d neighbors and %f radius\n", min_neighbors, radius);

	// set params and apply filter
        ror.setMinNeighborsInRadius(min_neighbors);
	ror.setRadiusSearch(radius);
	ror.filter(local_indices);

        PCL_INFO("      %d points filtered to %d following outlier removal\n", indices.size(), local_indices.size());
      }
      else if (name == "VoxelGrid")
      {
	// initial setup
        pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(local_cloud);
	//vg.setIndices(temp_indices);

	// parse params
	float x = vt.second.get<float>("setLeafSize.x", 1.0);
	float y = vt.second.get<float>("setLeafSize.y", 1.0);
	float z = vt.second.get<float>("setLeafSize.z", 1.0);

	// summarize settings
	PCL_DEBUG("      leaf size: %f, %f, %f\n", x, y, z);

	// set params and apply filter
	vg.setLeafSize(x, y, z);
	//vg.filter(local_indices);
      }
      else if (name == "ProgressiveMorphologicalFilter")
      {
        PCL_DEBUG( "pmf\n" );
        pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pmf.setInputCloud(local_cloud);
	//pmf.setIndices(temp_indices);
	pmf.extract(local_indices);

        PCL_INFO("      %d points filtered to %d following progressive morphological filter\n", indices.size(), local_indices.size());
      }
      else
      {
        PCL_WARN("Requested filter `%s` not implemented! Skipping...\n", name.c_str());
      }

      // need to update indices with local_indices
      std::vector<int> temp_indices(local_indices.size());
      for ( int idx = 0; idx < local_indices.size(); ++idx )
        temp_indices[idx] = indices[local_indices[idx]];
      indices.swap(temp_indices);
    }

    PCL_INFO("\n");
  }
  catch (std::exception const& e)
  {
    PCL_ERROR("[pcl::%s::applyFilterIndices] Error parsing JSON and creating pipeline! %s\n", getClassName().c_str(), e.what());
  }

  oii = indices.size();

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_Pipeline(T) template class PCL_EXPORTS pcl::Pipeline<T>;

#endif  // PCL_FILTERS_IMPL_PIPELINE_HPP_

