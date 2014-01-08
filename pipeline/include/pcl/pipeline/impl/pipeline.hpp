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

#ifndef PCL_FILTERS_IMPL_PIPELINE_HPP_
#define PCL_FILTERS_IMPL_PIPELINE_HPP_

#include <exception>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pipeline/pipeline.h>

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

    std::cout << std::endl;
    std::cout << "Processing " << filename_ << std::endl;
    std::cout << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "NAME:    " << pt.get<std::string>("pipeline.name","") << std::endl;
    std::cout << "HELP:    " << pt.get<std::string>("pipeline.help","") << std::endl;
    std::cout << "VERSION: " << pt.get<std::string>("pipeline.version","") << std::endl;
    std::cout << "AUTHOR:  " << pt.get<std::string>("pipeline.author","") << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    int step = 1;

    BOOST_FOREACH(boost::property_tree::ptree::value_type &vt, pt.get_child("pipeline.filters"))
    {
      std::cout << std::endl;
      std::string name = vt.second.get<std::string>("name","");
      std::string help = vt.second.get<std::string>("help","");

      std::cout << "   Step " << step++ << ") " << name << ": " << help << std::endl;

      if (name == "PassThrough")
      {
	pcl::PassThrough<PointT> pass;
        pass.setInputCloud(input_);
        pass.setIndices(indices_);

        std::string field = vt.second.get<std::string>("setFilterFieldName");
        std::cout << "      Field name: " << field << std::endl;
	pass.setFilterFieldName(field);

	float m1 = vt.second.get<float>("setFilterLimits.min", -std::numeric_limits<float>::max());
	float m2 = vt.second.get<float>("setFilterLimits.max", std::numeric_limits<float>::max());
        std::cout << "      Limits: " << m1 << ", " << m2 << std::endl;
	pass.setFilterLimits(m1, m2);

        pass.filter(indices);
      }

      if (name == "StatisticalOutlierRemoval")
      {
        pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(input_);
	sor.setIndices(indices_);

	int nr_k = vt.second.get<int>("setMeanK", 2);
        sor.setMeanK(nr_k);

	double stddev_mult = vt.second.get<double>("setStddevMulThresh", 0.0);
	sor.setStddevMulThresh(stddev_mult);

	std::cout << "       " << nr_k << " neighbors and " << stddev_mult << " multiplier" << std::endl;

	sor.filter(indices);
      }

      if (name == "VoxelGrid")
      {
        pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(input_);
	vg.setIndices(indices_);

	float x = vt.second.get<float>("setLeafSize.x", 1.0);
	float y = vt.second.get<float>("setLeafSize.y", 1.0);
	float z = vt.second.get<float>("setLeafSize.z", 1.0);
	std::cout << "      leaf size: " << x << ", " << y << ", " << z << std::endl;
	vg.setLeafSize(x, y, z);

	//vg.filter(indices);
      }
    }

    std::cout << std::endl;
  }
  catch (std::exception const& e)
  {
    std::cerr << e.what() << std::endl;
  }

  oii = indices.size();

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_Pipeline(T) template class PCL_EXPORTS pcl::Pipeline<T>;

#endif  // PCL_FILTERS_IMPL_PIPELINE_HPP_

