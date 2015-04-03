/*!
 * \file SegmentationZone.h
 * \brief The criteria for a segmentation zone.
 *
 * Segmentation zones are defined by X, Y, and Z bounds for use in segmenation. There are determined by bounds placed
 * on the RPY of a given transform. Each zone also has associated frame information.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 17, 2015
 */

#ifndef RAIL_SEGMENTATION_SEGMENTATION_ZONE_H_
#define RAIL_SEGMENTATION_SEGMENTATION_ZONE_H_

// C++ Standard Library
#include <string>

namespace rail
{
namespace segmentation
{

/*!
 * \class SegmentationZone
 * \brief The criteria for a segmentation zone.
 *
 * Segmentation zones are defined by X, Y, and Z bounds for use in segmenation. There are determined by bounds placed
 * on the RPY of a given transform. Each zone also has associated frame information.
 */
class SegmentationZone
{
public:
  /*!
   * \brief Create a SegmentationZone with the given frame information.
   *
   * Creates a new segmenation zone with the given frame information. Default minimum values will be -infinity and
   * default maximum values will be infinity. The remove surface flag also defaults to true.
   *
   * \param name The name of the zone (defaults to the empty string).
   * \param parent_frame_id The parent frame of the transform to monitor (defaults to the empty string).
   * \param child_frame_id The child frame of the transform to monitor (defaults to the empty string).
   * \param bounding_frame_id The frame the X, Y, and Z bounds are defined in (defaults to the empty string).
   * \param segmentation_frame_id The frame that segmented objects should be defined in (defaults to the empty string).
   */
  SegmentationZone(const std::string &name = "", const std::string &parent_frame_id = "",
      const std::string &child_frame_id = "", const std::string &bounding_frame_id = "",
      const std::string &segmentation_frame_id = "");

  /*!
   * \brief Remove surface value mutator.
   *
   * Set the remove surface value of this SegmentationZone.
   *
   * \param remove_surface The new remove surface value.
   */
  void setRemoveSurface(const bool remove_surface);

  /*!
   * \brief Remove surface value accessor.
   *
   * Check if a surface detection and removal should be performed in this zone.
   *
   * \return The remove surface value value.
   */
  bool getRemoveSurface() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the name value of this SegmentationZone.
   *
   * \param name The new name ID value.
   */
  void setName(const std::string &name);

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this SegmentationZone.
   *
   * \return The name value.
   */
  const std::string &getName() const;

  /*!
   * \brief Parent frame ID value mutator.
   *
   * Set the parent frame ID value of this SegmentationZone.
   *
   * \param parent_frame_id The new parent frame ID value.
   */
  void setParentFrameID(const std::string &parent_frame_id);

  /*!
   * \brief Parent frame ID value accessor.
   *
   * Get the parent frame ID value of this SegmentationZone.
   *
   * \return The parent frame ID value.
   */
  const std::string &getParentFrameID() const;

  /*!
   * \brief Child frame ID value mutator.
   *
   * Set the child frame ID value of this SegmentationZone.
   *
   * \param child_frame_id The new child frame ID value.
   */
  void setChildFrameID(const std::string &child_frame_id);

  /*!
   * \brief Child frame ID value accessor.
   *
   * Get the child frame ID value of this SegmentationZone.
   *
   * \return The child frame ID value.
   */
  const std::string &getChildFrameID() const;

  /*!
   * \brief Segmentation frame ID value mutator.
   *
   * Set the segmentation frame ID value of this SegmentationZone.
   *
   * \param segmentation_frame_id The new segmentation frame ID value.
   */
  void setSegmentationFrameID(const std::string &segmentation_frame_id);

  /*!
   * \brief Segmentation frame ID value accessor.
   *
   * Get the segmentation frame ID value of this SegmentationZone.
   *
   * \return The segmentation frame ID value.
   */
  const std::string &getSegmentationFrameID() const;

  /*!
   * \brief Bounding frame ID value mutator.
   *
   * Set the bounding frame ID value of this SegmentationZone.
   *
   * \param bounding_frame_id The new bounding frame ID value.
   */
  void setBoundingFrameID(const std::string &bounding_frame_id);

  /*!
   * \brief Segmentation frame ID value accessor.
   *
   * Get the bounding frame ID value of this SegmentationZone.
   *
   * \return The bounding frame ID value.
   */
  const std::string &getBoundingFrameID() const;

  /*!
   * \brief Roll min value mutator.
   *
   * Set the roll min of this SegmentationZone.
   *
   * \param roll_min The new roll min ID value.
   */
  void setRollMin(const double roll_min);

  /*!
   * \brief Roll min value accessor.
   *
   * Get the roll min value of this SegmentationZone.
   *
   * \return The roll min value.
   */
  double getRollMin() const;

  /*!
   * \brief Roll max value mutator.
   *
   * Set the roll max of this SegmentationZone.
   *
   * \param roll_max The new roll max ID value.
   */
  void setRollMax(const double roll_max);

  /*!
   * \brief Roll max value accessor.
   *
   * Get the roll max value of this SegmentationZone.
   *
   * \return The roll max value.
   */
  double getRollMax() const;

  /*!
   * \brief Pitch min value mutator.
   *
   * Set the pitch min of this SegmentationZone.
   *
   * \param pitch_min The new pitch min ID value.
   */
  void setPitchMin(const double pitch_min);

  /*!
   * \brief Pitch min value accessor.
   *
   * Get the pitch min value of this SegmentationZone.
   *
   * \return The pitch min value.
   */
  double getPitchMin() const;

  /*!
   * \brief Pitch max value mutator.
   *
   * Set the pitch max of this SegmentationZone.
   *
   * \param pitch_max The new pitch max ID value.
   */
  void setPitchMax(const double pitch_max);

  /*!
   * \brief Pitch max value accessor.
   *
   * Get the pitch max value of this SegmentationZone.
   *
   * \return The pitch max value.
   */
  double getPitchMax() const;

  /*!
   * \brief Yaw min value mutator.
   *
   * Set the yaw min of this SegmentationZone.
   *
   * \param yaw_min The new yaw min ID value.
   */
  void setYawMin(const double yaw_min);

  /*!
   * \brief Yaw min value accessor.
   *
   * Get the yaw min value of this SegmentationZone.
   *
   * \return The yaw min value.
   */
  double getYawMin() const;

  /*!
   * \brief Yaw max value mutator.
   *
   * Set the yaw max of this SegmentationZone.
   *
   * \param yaw_max The new yaw max ID value.
   */
  void setYawMax(const double yaw_max);

  /*!
   * \brief Yaw max value accessor.
   *
   * Get the yaw max value of this SegmentationZone.
   *
   * \return The yaw max value.
   */
  double getYawMax() const;

  /*!
   * \brief X min value mutator.
   *
   * Set the x min of this SegmentationZone.
   *
   * \param x_min The new x min ID value.
   */
  void setXMin(const double x_min);

  /*!
   * \brief X min value accessor.
   *
   * Get the x min value of this SegmentationZone.
   *
   * \return The x min value.
   */
  double getXMin() const;

  /*!
   * \brief X max value mutator.
   *
   * Set the x max of this SegmentationZone.
   *
   * \param x_max The new x max ID value.
   */
  void setXMax(const double x_max);

  /*!
   * \brief X max value accessor.
   *
   * Get the x max value of this SegmentationZone.
   *
   * \return The x max value.
   */
  double getXMax() const;

  /*!
   * \brief Y min value mutator.
   *
   * Set the y min of this SegmentationZone.
   *
   * \param y_min The new y min ID value.
   */
  void setYMin(const double y_min);

  /*!
   * \brief Y min value accessor.
   *
   * Get the y min value of this SegmentationZone.
   *
   * \return The y min value.
   */
  double getYMin() const;

  /*!
   * \brief Y max value mutator.
   *
   * Set the y max of this SegmentationZone.
   *
   * \param y_max The new y max ID value.
   */
  void setYMax(const double y_max);

  /*!
   * \brief Y max value accessor.
   *
   * Get the y max value of this SegmentationZone.
   *
   * \return The y max value.
   */
  double getYMax() const;

  /*!
   * \brief Z min value mutator.
   *
   * Set the z min of this SegmentationZone.
   *
   * \param z_min The new z min ID value.
   */
  void setZMin(const double z_min);

  /*!
   * \brief Z min value accessor.
   *
   * Get the z min value of this SegmentationZone.
   *
   * \return The z min value.
   */
  double getZMin() const;

  /*!
   * \brief Z max value mutator.
   *
   * Set the z max of this SegmentationZone.
   *
   * \param z_max The new z max ID value.
   */
  void setZMax(const double z_max);

  /*!
   * \brief Z max value accessor.
   *
   * Get the z max value of this SegmentationZone.
   *
   * \return The z max value.
   */
  double getZMax() const;

private:
  /*! If a surface removal should be done. */
  bool remove_surface_;
  /*! The associated name and frame information for this zone. */
  std::string name_, parent_frame_id_, child_frame_id_, segmentation_frame_id_, bounding_frame_id_;
  /*! The limits for this zone. */
  double roll_min_, roll_max_, pitch_min_, pitch_max_, yaw_min_, yaw_max_, x_min_, x_max_, y_min_, y_max_, z_min_,
      z_max_;
};

}
}

#endif
