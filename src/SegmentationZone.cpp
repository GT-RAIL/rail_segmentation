/*!
 * \file SegmentationZone.cpp
 * \brief The criteria for a segmentation zone.
 *
 * Segmentation zones are defined by X, Y, and Z bounds for use in segmenation. There are determined by bounds placed
 * on the RPY of a given transform. Each zone also has associated frame information.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 17, 2015
 */

// RAIL Segmentation
#include "rail_segmentation/SegmentationZone.h"

// C++ Standard Library
#include <limits>

using namespace std;
using namespace rail::segmentation;

SegmentationZone::SegmentationZone(const string &name, const string &parent_frame_id, const string &child_frame_id,
    const string &bounding_frame_id, const string &segmentation_frame_id)
    : name_(name), parent_frame_id_(parent_frame_id), child_frame_id_(child_frame_id),
      bounding_frame_id_(bounding_frame_id), segmentation_frame_id_(segmentation_frame_id)
{
  // default remove surface
  remove_surface_ = true;

  // set default limits
  roll_min_ = -numeric_limits<double>::infinity();
  pitch_min_ = -numeric_limits<double>::infinity();
  yaw_min_ = -numeric_limits<double>::infinity();
  x_min_ = -numeric_limits<double>::infinity();
  y_min_ = -numeric_limits<double>::infinity();
  z_min_ = -numeric_limits<double>::infinity();

  roll_max_ = numeric_limits<double>::infinity();
  pitch_max_ = numeric_limits<double>::infinity();
  yaw_max_ = numeric_limits<double>::infinity();
  x_max_ = numeric_limits<double>::infinity();
  y_max_ = numeric_limits<double>::infinity();
  z_max_ = numeric_limits<double>::infinity();
}

void SegmentationZone::setRemoveSurface(const bool remove_surface)
{
  remove_surface_ = remove_surface;
}

bool SegmentationZone::getRemoveSurface() const
{
  return remove_surface_;
}

void SegmentationZone::setName(const string &name)
{
  name_ = name;
}

const string &SegmentationZone::getName() const
{
  return name_;
}

void SegmentationZone::setParentFrameID(const string &parent_frame_id)
{
  parent_frame_id_ = parent_frame_id;
}

const string &SegmentationZone::getParentFrameID() const
{
  return parent_frame_id_;
}

void SegmentationZone::setChildFrameID(const string &child_frame_id)
{
  child_frame_id_ = child_frame_id;
}

const string &SegmentationZone::getChildFrameID() const
{
  return child_frame_id_;
}

void SegmentationZone::setSegmentationFrameID(const string &segmentation_frame_id)
{
  segmentation_frame_id_ = segmentation_frame_id;
}

const string &SegmentationZone::getSegmentationFrameID() const
{
  return segmentation_frame_id_;
}

void SegmentationZone::setBoundingFrameID(const string &bounding_frame_id)
{
  bounding_frame_id_ = bounding_frame_id;
}

const string &SegmentationZone::getBoundingFrameID() const
{
  return bounding_frame_id_;
}

void SegmentationZone::setRollMin(const double roll_min)
{
  roll_min_ = roll_min;
}

double SegmentationZone::getRollMin() const
{
  return roll_min_;
}

void SegmentationZone::setRollMax(const double roll_max)
{
  roll_max_ = roll_max;
}

double SegmentationZone::getRollMax() const
{
  return roll_max_;
}

void SegmentationZone::setPitchMin(const double pitch_min)
{
  pitch_min_ = pitch_min;
}

double SegmentationZone::getPitchMin() const
{
  return pitch_min_;
}

void SegmentationZone::setPitchMax(const double pitch_max)
{
  pitch_max_ = pitch_max;
}

double SegmentationZone::getPitchMax() const
{
  return pitch_max_;
}

void SegmentationZone::setYawMin(const double yaw_min)
{
  yaw_min_ = yaw_min;
}

double SegmentationZone::getYawMin() const
{
  return yaw_min_;
}

void SegmentationZone::setYawMax(const double yaw_max)
{
  yaw_max_ = yaw_max;
}

double SegmentationZone::getYawMax() const
{
  return yaw_max_;
}


void SegmentationZone::setXMin(const double x_min)
{
  x_min_ = x_min;
}

double SegmentationZone::getXMin() const
{
  return x_min_;
}

void SegmentationZone::setXMax(const double x_max)
{
  x_max_ = x_max;
}

double SegmentationZone::getXMax() const
{
  return x_max_;
}

void SegmentationZone::setYMin(const double y_min)
{
  y_min_ = y_min;
}

double SegmentationZone::getYMin() const
{
  return y_min_;
}

void SegmentationZone::setYMax(const double y_max)
{
  y_max_ = y_max;
}

double SegmentationZone::getYMax() const
{
  return y_max_;
}

void SegmentationZone::setZMin(const double z_min)
{
  z_min_ = z_min;
}

double SegmentationZone::getZMin() const
{
  return z_min_;
}

void SegmentationZone::setZMax(const double z_max)
{
  z_max_ = z_max;
}

double SegmentationZone::getZMax() const
{
  return z_max_;
}
