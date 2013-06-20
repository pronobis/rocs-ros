//
// = FILENAME
//    InDoorOpeningDetector.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef InDoorOpeningDetector_hh
#define InDoorOpeningDetector_hh

#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <math.h>
#include <string.h>

#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

/**
 * Class that takes a laser scan and checks if the laser is likely to
 * be in a door opening. This can be used in conjunction with the
 * NavGraph to partition the graph into different areas.
 *
 * NOTE this class assumes that the laser is mounted facing forward on
 * the robot.
 */
class InDoorOpeningDetector {
public:
  /// Width of the last detected door
  double m_Width;

  // The distance to the door post on the right side as seen by the laser
  double m_RangeR;

  // The distance to the door post on the left side as seen by the laser
  double m_RangeL;

  // The angle in scan coordinate to the door post on the right side
  double m_AngleR;

  // The angle in scan coordinate to the door post on the left side
  double m_AngleL;

public:

  /**
   * @param minWidth minimum width of the opening to pass for a door
   * @param maxWidth maximum width of the opening to pass for a door
   */
  InDoorOpeningDetector(double minWidth = 0.6,
                        double maxWidth = 1.0);

  /**
   * Set the minium range that a laser reading can have and still be
   * classified as valid
   */
  void setMinRange(double r) { m_MinRange = r; }

  /**
   * Get the minium range that a laser reading can have and still be
   * classified as valid
   */
  double getMinRange() const { return m_MinRange; }

  /**
   * Use this function to select what algorithm to use to detect the door
   */
  void setUsedAlgorithm(int a) { m_Algorithm = a; }

  /**
   * Use this function to specify the max gap in range between two
   * points to say that they are connected when looking for connected pints
   */
  void setMaxGap(double g) { m_MaxGap = g; }

  /**
   * Use this function to specify how big a structuire need to be to
   * be considered a door post
   */
  void setMinDoorPostSize(double s) { m_MinPostSize = s; }

  /**
   * Use this function to specify how long a range reading from the
   * laser need to be between the door posts to say that it votes for
   * there being clearance enough forward to say it is a door. This is
   * meant to remve false door created by standing in a corner of a
   * room for example.
   */
  void setClearanceThreshold(double d) { m_ClearanceThreshold = d; }

  /**
   * The number of beams that need to be longer than clearance
   * threshold between teh door post to say that it is a door
   */
  void setMinNumLongForClearance(int n) { m_MinNumLongEnoughForClearance = n; }

  /**
   * @return true if we are likely to be in a door opening. The
   * threshods for the widht of the opening is taken from the
   * constructor argument.
   */
  bool inDoorOpening(const sensor_msgs::LaserScan &scan);

  /**
   * Same as above but you can specify the thresholds for thw door
   * width in the call.
   */
  bool inDoorOpening(const sensor_msgs::LaserScan &scan, double minWidth, double maxwidth);

protected:

  /**
   * The original algorithm
   */
  bool algorithm1(const sensor_msgs::LaserScan &scan, double minWidth, double maxwidth);

  /**
   * Looks for closest points to the left and right and then checks if
   * these distances are at the right distance from each other and
   * that there are enough points on each side to support it being a
   * real door and not just two chairs for example.
   */
  bool algorithm2(const sensor_msgs::LaserScan &scan, double minWidth, double maxwidth);

  /**
   * This function checks if there is enough clearance forward between
   * two indices for these to be credible candidate for door
   * posts. This is to avoid detecting door posts in corners of a room
   * for example, when the laser might tell you that there are things
   * on the right distances on the sides.
   */
  bool enoughClearanceForward(const sensor_msgs::LaserScan &scan, int start, int end);

  /**
   * @return distance to the point farthest away from the point with
   * index index where no point in the chain can be further away from
   * the previous than maxGap
   */
  double connectedDistance(const sensor_msgs::LaserScan &scan, int index, double maxGap);

  /*

  /// Find the index (index) of the point closest to another point
  /// (target) but still connected (all gaps <= maxGap) to yet another
  /// point (start).
  ///
  /// This function is used when the poinst closest to the robot have
  /// been identified. Now one wants to check for the poinst closest to
  /// each other along the structire that they blong to (hence a small
  /// gap between the points). If we do this with index1 and index2
  /// being the left and right and the the right and left side of the
  /// door we should get the smallest part of the door opening which is
  /// likely to best define the actual door
  double getClosestConnectedPointTo(const sensor_msgs::LaserScan &scan,
                                    double maxGap,
                                    int target, int start,
                                    int &index);


  /// Calculate the cartesian distance between points with index1 and index2.  
  /// NOTE for reasons of speed no error checking is done
  double getPointDistance(const sensor_msgs::LaserScan &scan, int index1, int index2)
  {
    double x1 = scan(index1) * cos(scan.getStartAngle() +
                                   scan.getAngleStep() * index1);
    double y1 = scan(index1) * sin(scan.getStartAngle() +
                                   scan.getAngleStep() * index1);
    double x2 = scan(index2) * cos(scan.getStartAngle() +
                                   scan.getAngleStep() * index2);
    double y2 = scan(index2) * sin(scan.getStartAngle() +
                                   scan.getAngleStep() * index2);
    return hypot(y2 - y1, x2 - x1);
  }

  */

  double m_MinWidth;
  double m_MaxWidth;

  int m_Algorithm;

  double m_MinRange;

  double m_MaxGap;
  double m_MinPostSize;

  double m_ClearanceThreshold;
  int m_MinNumLongEnoughForClearance;
}; // class InDoorOpeningDetector

#endif // InDoorOpeningDetector_hh
