# Geometry

The geometry folder contains classes that handle fundamental geometric representations and transformations commonly used in applications of GTSAM.
The `geometry` module collects the Lie groups, point classes, and camera models that form the foundation of many algorithms in GTSAM. These types provide representations for poses, rotations, and calibration models used across inference and SLAM applications.

## Core Types

- [Rot2](doc/Rot2.ipynb) and [Rot3](doc/Rot3.ipynb): Represent 2D and 3D rotation groups.
- [Pose2](doc/Pose2.ipynb) and [Pose3](doc/Pose3.ipynb): Rigid body poses in 2D and 3D.
- [Point2](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Point2.h) and [Point3](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Point3.h): Basic 2D and 3D point containers.
- [Unit3](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Unit3.h): Direction vectors with unit length.
- [Similarity2](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Similarity2.h) and [Similarity3](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Similarity3.h): Similarity transformations combining rotation, translation, and scale.
- [SO3](doc/SO3.ipynb), [SO4](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SO4.h), and [SOn](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SOn.h): General special orthogonal group implementations.
- [Quaternion](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Quaternion.h): Quaternion wrapper for rotations.

## Camera Models

- Intrinsic calibration types such as [Cal3_S2](doc/Cal3_S2.ipynb), [Cal3Bundler](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Cal3Bundler.h), [Cal3_S2Stereo](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Cal3_S2Stereo.h), [Cal3Fisheye](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Cal3Fisheye.h), and [Cal3Unified](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Cal3Unified.h).
- Camera wrappers including [PinholeCamera](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/PinholeCamera.h), [PinholePose](doc/PinholePose.ipynb), [CalibratedCamera](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/CalibratedCamera.h), [SimpleCamera](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SimpleCamera.h), [StereoCamera](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/StereoCamera.h), and [SphericalCamera](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SphericalCamera.h).
- Associated measurement types such as [StereoPoint2](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/StereoPoint2.h) and utilities for working with sets of cameras ([PinholeSet](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/PinholeSet.h), [CameraSet](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/CameraSet.h)).

## Geometric Relations and Utilities

- [BearingRange](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/BearingRange.h): Relates bearing and range measurements.
- [EssentialMatrix](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/EssentialMatrix.h) and [FundamentalMatrix](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/FundamentalMatrix.h): Two-view geometric relations.
- [Line3](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Line3.h) and [OrientedPlane3](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/OrientedPlane3.h): Representations for lines and planes in 3D.
- Triangulation and vision utilities ([triangulation](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/triangulation.h), [Event](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Event.h)).

These classes provide the building blocks for constructing factors and performing computations throughout the rest of GTSAM.
