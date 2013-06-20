//
// = FILENAME
//    InDoorOpeningDetector.cc
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

#include "InDoorOpeningDetector.h"

#include <math.h>

InDoorOpeningDetector::InDoorOpeningDetector(double minWidth,
                                             double maxWidth)
  :m_Width(1.0),
   m_RangeR(0.5),
   m_RangeL(0.5),
   m_MinWidth(minWidth),
   m_MaxWidth(maxWidth)
{
  m_Algorithm = 1;

  m_MaxGap = 0.02;
  m_MinPostSize = 0.05;
  m_ClearanceThreshold = 1.5;
  m_MinNumLongEnoughForClearance = 50;

  m_MinRange = 0.02;
}

bool 
InDoorOpeningDetector::inDoorOpening(const sensor_msgs::LaserScan &scan)
{
  return inDoorOpening(scan, m_MinWidth, m_MaxWidth);
}

bool 
InDoorOpeningDetector::inDoorOpening(const sensor_msgs::LaserScan &scan,
                                     double minWidth, double maxWidth)
{
  if (m_Algorithm == 2) return algorithm2(scan, minWidth, maxWidth);
  else return algorithm1(scan, minWidth, maxWidth);
}

bool 
InDoorOpeningDetector::algorithm1(const sensor_msgs::LaserScan &scan,
                                  double minWidth, double maxWidth)
{
  int nsum = 10;
  double sumR = 0, sumL = 0;

  // Since the robot might not run parallel through every door we
  // cannot check straight to the side only. As a simple hack we assume
  // that the door frame is detected at one end of the scan and then
  // we search from the other end towards the middle for a chunk of
  // points that are at the right distance from the other end. For
  // small angles we do not care about compensating for not really
  // measuring a straight line.
  int maxShift = int(30.0 / (180.0 / M_PI * scan.angle_increment) + 0.5);

  // Add extra shift if the scan if more than 180 degs
  double fov = scan.angle_increment * (scan.ranges.size()-1);
  if (fov > M_PI) {
    maxShift += int(0.5*(fov-M_PI) / 180.0 / M_PI * scan.angle_increment);
  }

  // Loop over the shift in to the scan
  bool foundDoor = false;
  m_Width = 1.0;
  m_RangeR = 0.5 * m_Width;
  m_RangeL = 0.5 * m_Width;
  m_AngleR = scan.angle_min;
  m_AngleL = scan.angle_min + fov;

  for (int j = 0; j <= maxShift; j++) {
    
    // Start with assuming that the right side is seen at the end of
    // the scan and look for the left side progressively more into the
    // scan starting from the other end
    sumR = 0; 
    sumL = 0;
    int n = 0;
    for (int i = 0; i < nsum; i++) {
      if (scan.ranges.at(i) > m_MinRange &&
          scan.ranges.at(scan.ranges.size() - 1 - i - j) > m_MinRange) {
        sumR += scan.ranges.at(i);
        sumL += scan.ranges.at(scan.ranges.size() - 1 - i - j);
        n++;
      }
    }

    if (n > 0) {
      sumR /= n;
      sumL /= n;
      m_RangeR = sumR;
      m_RangeL = sumL;
      m_AngleR = scan.angle_min + scan.angle_increment*(nsum/2);
      m_AngleL = scan.angle_min + fov - scan.angle_increment*(j+nsum/2);
      m_Width = sumR + sumL;
      if (minWidth <= m_Width && m_Width <= maxWidth) {
        
        foundDoor = enoughClearanceForward(scan, 0, scan.ranges.size() - 1 - j);

        break;
      }
    }

    if (j == 0) {
      // If the distance is too long in both directions there is no
      // point in continueing at all
      if (sumR > maxWidth && sumL > maxWidth) break;

      // No need to shift both left and right side 0 steps, it is the same
      continue;
    }

    // Now assume that the left side is seen at the end of the scan
    // and look for the right side progressively more into the scan
    // starting from the other end
    sumR = 0; 
    sumL = 0;
    n = 0;
    for (int i = 0; i < nsum; i++) {

      if (scan.ranges.at(i+j) > m_MinRange &&
          scan.ranges.at(scan.ranges.size() - 1 - i) > m_MinRange) {
        sumR += scan.ranges.at(i+j);
        sumL += scan.ranges.at(scan.ranges.size() - 1 - i);
        n++;
      }
    }

    if (n > 0) {
      sumR /= nsum;
      sumL /= nsum;
      m_Width = sumR + sumL;
      m_RangeR = sumR;
      m_RangeL = sumL;
      m_AngleR = scan.angle_min + scan.angle_increment*(j+nsum/2);
      m_AngleL = scan.angle_min + fov - scan.angle_increment*(nsum/2);
      
      if (minWidth <= m_Width && m_Width <= maxWidth) {
        foundDoor = enoughClearanceForward(scan, j, scan.ranges.size() - 1);
        break;
      }
    }
  }

  ROS_DEBUG("maxShift: %d sumR= %f sumL= %f width= %f\n",maxShift,sumR,sumL,m_Width);

  return foundDoor;
}

bool 
InDoorOpeningDetector::algorithm2(const sensor_msgs::LaserScan &scan,
                                  double minWidth, double maxWidth)
{
  // We search on each side a certain angle out for the closest point
  const double maxAngle = 20.0 * M_PI / 180.0;

  // Search for the point closest to the scanner on the right hand side
  double a = scan.angle_min;
  int minRi = -1;
  double minRd = 1e10;
  double minRa = -1;
  int i = 0;
  while (a < maxAngle) {
    if (scan.ranges.at(i) < minRd && scan.ranges.at(i) >= m_MinRange) {
      minRa = a;
      minRd = scan.ranges.at(i);
      minRi = i;
    }

    i++;
    a += scan.angle_increment;
  }
  
  // Search for the point closest to the scanner on the left hand side
  double endAngle = (scan.angle_min +
                     (scan.ranges.size()-1) * scan.angle_increment);
  a = endAngle;
  int minLi = -1;
  double minLd = 1e10;
  double minLa = -1;
  i = scan.ranges.size()-1;
  while (a > endAngle - maxAngle) {
    if (scan.ranges.at(i) < minLd && scan.ranges.at(i) >= m_MinRange) {
      minLa = a;
      minLd = scan.ranges.at(i);
      minLi = i;
    }

    i--;
    a -= scan.angle_increment;
  }

  if (!enoughClearanceForward(scan, minRi, minLi)) {
//    CureCERR(50) << "Not enough clearance forward discarding door\n";
    return false;
  }

  double w = minRd + minLd;

//  CureCERR(60) << "w=" << w << " minRd=" << minRd << " minLd=" << minLd
//               << std::endl;

  // Check if the points found are outside the valid door width values
  if (w < minWidth || maxWidth < w) {
//    CureCERR(50) << "Door width out of bounds w="
//                 << w << std::endl;
    return false;
  }

  if (connectedDistance(scan, minRi, m_MaxGap) < m_MinPostSize) {
//    CureCERR(50) << "Not enough support on right side for a door\n";
    return false;
  }
  if (connectedDistance(scan, minLi, m_MaxGap) < m_MinPostSize) {
//    CureCERR(50) << "Not enough support on left side for a door\n";
    return false;  
  }

  m_RangeR = minRd;
  m_RangeL = minLd;
  m_AngleR = minRa;
  m_AngleL = minLa;

  // Calculate the position of the door

//  CureCERR(40) << "Found a door\n";
  return true;  
}

double
InDoorOpeningDetector::connectedDistance(const sensor_msgs::LaserScan &scan,
                                         int index,
                                         double maxGap)
{
  if (index < 0 || index >= scan.ranges.size()) {
//    CureCERR(20) << "Cannot check connectedDistance, index out of bounds\n";
    return 0;
  } 

  double x0 = scan.ranges.at(index) * cos(scan.angle_min +
                                scan.angle_increment * index);
  double y0 = scan.ranges.at(index) * sin(scan.angle_min +
                                scan.angle_increment * index);

  // First we search to lower indices
  double dist1 = 0;
  int i = index - 1;
  while (i >= 0) {
    if (fabs(scan.ranges.at(i) - scan.ranges.at(i+1)) > maxGap) {
      double x1 = scan.ranges.at(i+1) * cos(scan.angle_min +
                                  scan.angle_increment * (i+1));
      double y1 = scan.ranges.at(i+1) * sin(scan.angle_min +
                                  scan.angle_increment * (i+1));
      dist1 = hypot(y1-y0, x1-x0);
      break;
    }
    i--;
  }
  if (i < 0) {
    double x1 = scan.ranges.at(0) * cos(scan.angle_min);
    double y1 = scan.ranges.at(0) * sin(scan.angle_min);
    dist1 = hypot(y1-y0, x1-x0);
  }

  // Then we search to higher indices
  double dist2 = 0;
  i = index + 1;
  while (i < scan.ranges.size()) {
    if (fabs(scan.ranges.at(i) - scan.ranges.at(i-1)) > maxGap) {
      double x1 = scan.ranges.at(i-1) * cos(scan.angle_min +
                                  scan.angle_increment * (i-1));
      double y1 = scan.ranges.at(i-1) * sin(scan.angle_min +
                                  scan.angle_increment * (i-1));
      dist2 = hypot(y1-y0, x1-x0);
    }
    i++;
  }
  if (i >= scan.ranges.size()) {
    int n = scan.ranges.size() - 1;
    double x1 = scan.ranges.at(n) * cos(scan.angle_min +
                                scan.angle_increment * n);
    double y1 = scan.ranges.at(n) * sin(scan.angle_min +
                                scan.angle_increment * n);
    dist2 = hypot(y1-y0, x1-x0);
  }
  
  if (dist1 > dist2) return dist1;
  else return dist2;
}

/*
double
InDoorOpeningDetector::getClosestConnectedPointTo(const sensor_msgs::LaserScan &scan,
                                                  double maxGap,
                                                  int target, int start,
                                                  int &index)
{
  if (target < 0 || scan.ranges.size() <= target) {
    CureCERR(20) << "target index out of bounds\n";
    return start;
  }

  if (start < 0 || scan.ranges.size() <= start) {
    CureCERR(20) << "start index out of bounds\n";
    return start;
  }

  // First we search to lower indices
  double minD1 = getPointDistance(scan, target, start);
  int minI1 = start;
  int i = start - 1;
  while (i >= 0) {
    if (fabs(scan.ranges.at(i) - scan.ranges.at(i+1)) > maxGap) break;
    double d = getPointDistance(scan, target, i);
    if (d < minD1) {
      minD1 = d;
      minI1 = i;
    }
    i--;
  }

  // Then we search to higher indices
  double minD2 = getPointDistance(scan, target, start);
  int minI2 = start;
  i = start + 1;
  while (i < scan.ranges.size()) {
    if (fabs(scan.ranges.at(i) - scan.ranges.at(i-1)) > maxGap) break;
    double d = getPointDistance(scan, target, i);
    if (d < minD2) {
      minD2 = d;
      minI2 = i;
    }
    i--;
  }
  
  if (minD1 < minD2) {
    index = minI1;
    return minD1;
  } else {
    index = minI2;
    return minD2;
  }
}
*/

bool
InDoorOpeningDetector::enoughClearanceForward(const sensor_msgs::LaserScan &scan,
                                              int start, int end)
{
  int n = 0;
  for (int i = start; i <= end; i++) {
    if (scan.ranges.at(i) > m_ClearanceThreshold) n++;
  }

  bool enoughClearance = (n >= m_MinNumLongEnoughForClearance);
  if (enoughClearance) {
//    CureCERR(60) << "Found enough clearance for it to be a door\n";
  } else {
//    CureCERR(60) << "NOT enough clearance for it to be a door\n";
  }
  
  return enoughClearance;
}
