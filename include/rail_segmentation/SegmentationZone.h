#ifndef RAIL_SEGMENTATION_SEGMENTATION_ZONE_H_
#define RAIL_SEGMENTATION_SEGMENTATION_ZONE_H_

#include <string>

namespace rail
{
namespace segmentation
{

class SegmentationZone
{
public:
  SegmentationZone(const std::string &name = "", const std::string &parent_frame_id = "",
      const std::string &child_frame_id = "", const std::string &bounding_frame_id = "",
      const std::string &segmentation_frame_id = "");

  void setRemoveSurface(const bool remove_surface);

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

  void setRollMin(const double roll_min);

  double getRollMin() const;

  void setRollMax(const double roll_max);

  double getRollMax() const;

  void setPitchMin(const double pitch_min);

  double getPitchMin() const;

  void setPitchMax(const double pitch_max);

  double getPitchMax() const;

  void setYawMin(const double yaw_min);

  double getYawMin() const;

  void setYawMax(const double yaw_max);

  double getYawMax() const;

  void setXMin(const double x_min);

  double getXMin() const;

  void setXMax(const double x_max);

  double getXMax() const;

  void setYMin(const double y_min);

  double getYMin() const;

  void setYMax(const double y_max);

  double getYMax() const;

  void setZMin(const double z_min);

  double getZMin() const;

  void setZMax(const double z_max);

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
