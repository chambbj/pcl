.. _pcl_json_specification:

.. sectnum::

============================
Draft PCL JSON Specification
============================

:Author: Bradley J. Chambers (RadiantBlue Technologies, Inc.)
:Revision: 0.1
:Date: 28 February 2014
:Copyright: Copyright (c) 2014, RadiantBlue Technologies, Inc. This work is licensed under a Creative Commons Attribution 3.0 United States License.

The PCL JSON specification is a point cloud processing pipeline interchange
format based on JavaScript Object Notation (JSON), drawing inspiration from
both GeoJSON and TopoJSON.

.. contents::
   :depth: 4
   :backlinks: none

============
Introduction
============

A PCL JSON object represents a processing pipeline.

A complete PCL JSON data structure is always an object (in JSON terms). In PCL
JSON, an object consists of a collection of name/value pairs -- also called
members. For each member, the name is always a string. Member values are either
a string, number, object, array or one of the literals: "true", "false", and
"null". An array consists of elements where each element is a value as
described above.

Examples
--------

A PCL JSON pipeline:

.. code-block:: json

   {
     "pipeline": {
       "name": "My cool pipeline",
       "filters": [{
         "name": "VoxelGrid",
         "setLeafSize": {
           "x": 1.0,
           "y": 1.0,
           "z": 1.0
         }
       }]
     }
   }

Definitions
-----------

* JavaScript Object Notation (JSON), and the terms object, name, value, array,
  and number, are defined in IETF RTC 4627, at
  http://www.ietf.org/rfc/rfc4627.txt.

* The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD",
  "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this documention are to
  be interpreted as described in IETF RFC 2119, at
  http://www.ietf.org/rfc/rfc2119.txt.

================
PCL JSON Objects
================

PCL JSON always consists of a single object. This object (referred to as the
PCL JSON object below) represents a processing pipeline.

* The PCL JSON object may have any number of members (name/value pairs).

* The PCL JSON object must have a "pipeline" object.

Pipeline Objects
----------------

* A pipeline may have a member with the name "name" whose value is a string.

* A pipeline may have a member with the name "help" whose value is a string.

* A pipeline may have a member with the name "version" whose value is a number.

* A pipeline must have a member with the name "filters" whose value is an array of filters.

Filters
.......

A pipeline must have a "filters" member whose value is an array of filters.

A filter is any of the PCL filters that has been exposed through the PCL pipeline class.

ApproximateProgressiveMorphologicalFilter
`````````````````````````````````````````

:pcl:`ApproximateProgressiveMorphologicalFilter <pcl::ApproximateProgressiveMorphologicalFilter>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "ApproximateProgressiveMorphologicalFilter",
         "setMaxWindowSize": 65,
         "setSlope": 0.7,
         "setMaxDistance": 10,
         "setInitialDistance": 0.3,
         "setCellSize": 1,
         "setBase": 2,
         "setExponential": false
       }]
     }
   }

setMaxWindowSize
  Set the maximum window size to be used for filtering ground returns.
  [default: 33]

setSlope
  Set the slope value to be used in computing the height threshold. [default:
  1]

setMaxDistance
  Set the maximum height above the parameterized ground surface to be
  considered a ground return. [default: 2.5]

setInitialDistance
  Set the initial height above the parameterized ground surface to be
  considered a ground return. [default: 0.15]

setCellSize
  Set the cell size. [default: 1]

setBase
  Set the base to be used in computing progressive window sizes. [default: 2]

setExponential
  Set flag indicating whether or not to exponentially grow window sizes.
  [default: true]

ConditionalRemoval
``````````````````

:pcl:`ConditionalRemoval <pcl::ConditionalRemoval>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "ConditionalRemoval",
         "normalZ": {
           "min": 0,
           "max": 0.95
         }
       }]
     }
   }

normalZ (min, max)
  Set the numerlical limits for filtering points based on the z component of
  their normal. [default: 0, FLT_MAX]

GridMinimum
```````````

:pcl:`GridMinimum <pcl::GridMinimum>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "GridMinimum",
         "setResolution": 2
       }]
     }
   }

setResolution
  Set the grid resolution. [default: 1.0]

NormalEstimation
````````````````

:pcl:`NormalEstimation <pcl::NormalEstimation>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "NormalEstimation",
         "setRadiusSearch": 2
       }]
     }
   }

setKSearch
  Set the number of k nearest neighbors to use for the feature estimation. [default: 0]

setRadiusSearch
  Set the sphere radius that is to be used for determining the nearest neighbors used for the feature estimation. [default: 1]

PassThrough
```````````

:pcl:`PassThrough <pcl::PassThrough>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "PassThrough",
         "setFilterFieldName": "z",
          "setFilterLimits": {
            "min": 3850100,
            "max": 3850200
          }
       }]
     }
   }

setFilterFieldName
  Provide the name of the field to be used for filtering data.

setFilterLimits (min, max)
  Set the numerical limits for the field for filtering data. [default: +/-
  FLT_MAX]

ProgressiveMorphologicalFilter
``````````````````````````````

:pcl:`ProgressiveMorphologicalFilter <pcl::ProgressiveMorphologialFilter>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "ProgressiveMorphologicalFilter",
         "setMaxWindowSize": 65,
         "setSlope": 0.7,
         "setMaxDistance": 10,
         "setInitialDistance": 0.3,
         "setCellSize": 1,
         "setBase": 2,
         "setExponential": false
       }]
     }
   }

setMaxWindowSize
  Set the maximum window size to be used for filtering ground returns.
  [default: 33]

setSlope
  Set the slope value to be used in computing the height threshold. [default:
  1]

setMaxDistance
  Set the maximum height above the parameterized ground surface to be
  considered a ground return. [default: 2.5]

setInitialdistance
  Set the initial height above the parameterized ground surface to be
  considered a ground return. [default: 0.15]

setCellSize
  Set the cell size. [default: 1]

setBase
  Set the base to be used in computing progressive window sizes. [default: 2]

setExponential
  Set flag indicating whether or not to exponentially grow window sizes.
  [default: true]

RadiusOutlierRemoval
````````````````````

:pcl:`RadiusOutlierRemoval <pcl::RadiusOutlierRemoval>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "RadiusOutlierRemoval",
         "setMinNeighborsInRadius": 8,
         "setRadiusSearch": 1
       }]
     }
   }

setMinNeighborsInRadius
  Set the number of neighbors that need to be present in order to be
  classified as an inliear. [default: 2]

setRadiusSearch
  Set te radius of the sphere that will determine which points are neighbors.
  [default: 1.0]

StatisticalOutlierRemoval
`````````````````````````

:pcl:`StatisticalOutlierRemoval <pcl::StatisticalOutlierRemoval>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "StatisticalOutlierRemoval",
         "setMeanK": 8,
         "setStddevMulThresh": 1
       }]
     }
   }

setMeanK
  Set the number of nearest neighbors to use for mean distance estimation.
  [default: 2]

setStddevMulThresh
  Set the standard deviation multiplier for the distance threshold
  calculation. [default: 0.0]

VoxelGrid
`````````

:pcl:`VoxelGrid <pcl::VoxelGrid>`

.. code-block:: json

   {
     "pipeline": {
       "filters": [{
         "name": "VoxelGrid",
         "setLeafSize": {
           "x": 1.0,
           "y": 1.0,
           "z": 1.0
         }
       }]
     }
   }

setLeafSize (x, y, z)
  Set the voxel grid leaf size. [default: 1.0, 1.0, 1.0]
