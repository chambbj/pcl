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

PassThrough
```````````

:pcl:`PassThrough <pcl::PassThrough>`

setFilterFieldName
  TBD

setFilterLimits (min, max)
  TBD - default = +/- FLT_MAX

StatisticalOutlierRemoval
`````````````````````````

:pcl:`StatisticalOutlierRemoval <pcl::StatisticalOutlierRemoval>`

setMeanK
  default = 2

setStddevMulThresh
  default = 0.0

RadiusOutlierRemoval
````````````````````

:pcl:`RadiusOutlierRemoval <pcl::RadiusOutlierRemoval>`

setMinNeighborsInRadius
  default = 2

setRadiusSearch
  default = 1.0

GridMinimum
```````````

:pcl:`GridMinimum <pcl::GridMinimum>`

setLeafSize (x, y)
  default = 1.0

ProgressiveMorphologicalFilter
``````````````````````````````

:pcl:`ProgressiveMorphologicalFilter <pcl::ProgressiveMorphologialFilter>`
