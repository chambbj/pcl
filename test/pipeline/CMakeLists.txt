PCL_ADD_TEST(pipeline_pipeline test_pipeline
             FILES test_pipeline.cpp
             LINK_WITH pcl_gtest pcl_common pcl_io pcl_pipeline
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
   

