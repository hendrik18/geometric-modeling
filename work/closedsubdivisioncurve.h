#ifndef CLOSED_SUBDIVISION_CURVE_H
#define CLOSED_SUBDIVISION_CURVE_H

#include <parametrics/gmpcurve.h>
#include <core/containers/gmdvector.h>

class ClosedSubdivisionCurve : public GMlib::PCurve<float, 3>
{
  GM_SCENEOBJECT(ClosedSubdivisionCurve)

public:
  // Constructor
  ClosedSubdivisionCurve(const GMlib::DVector<GMlib::Vector<float, 3>> &controlPts, int degree)
      : _controlPoints(controlPts), _degree(degree)
  {

    // Constrain the parametric domain to [0, 1]
    this->setDomain(0.0f, 1.0f);

    // Compute subdivided points
    laneRiesenfeldSubdivision();
  }

  // Destructor
  ~ClosedSubdivisionCurve() override = default;

  // PCurve interface overrides
  void eval(float t, int d, bool left = true) const override;
  float getStartP() const override { return 0.0f; }
  float getEndP() const override { return 1.0f; }
  bool isClosed() const override { return true; } // Mark as closed

private:
  GMlib::DVector<GMlib::Vector<float, 3>> _controlPoints; // Original control polygon
  mutable GMlib::DVector<GMlib::Vector<float, 3>> _subdividedPoints;
  int _degree;

  void laneRiesenfeldSubdivision();
};

/*!
 *  eval(float t, int d, bool left) const
 *
 *  - Maps t in [0,1] to an index in _subdividedPoints.
 *  - Interpolates linearly between discrete points for a smooth curve.
 *  - Approximates the first derivative by finite differences if requested.
 */
void ClosedSubdivisionCurve::eval(float t, int d, bool /*left*/) const
{

  // Ensure _p has space for position + derivatives
  this->_p.setDim(d + 1);

  // Map t to [0, _subdividedPoints.getDim() - 1]
  float scaled_t = t * (_subdividedPoints.getDim() - 1);
  int index = static_cast<int>(std::floor(scaled_t)) % _subdividedPoints.getDim();
  float alpha = scaled_t - index; // Fractional part for interpolation

  // Interpolate between index and index+1 for a smooth result
  GMlib::Vector<float, 3> p1 = _subdividedPoints[index];
  GMlib::Vector<float, 3> p2 = _subdividedPoints[(index + 1) % _subdividedPoints.getDim()];

  this->_p[0] = (1.0f - alpha) * p1 + alpha * p2;

  // Approximate the first derivative if d > 0
  if (d > 0)
  {
    int next = (index + 1) % _subdividedPoints.getDim();
    int prev = (index - 1 + _subdividedPoints.getDim()) % _subdividedPoints.getDim();
    this->_p[1] = (_subdividedPoints[next] - _subdividedPoints[prev]) * 0.5f;
  }
}

/*!
 *  laneRiesenfeldSubdivision()
 *
 *  - Implements the standard Lane-Riesenfeld algorithm for a closed curve.
 *  - After generating the new points, the *last* point is forced to match the *first*,
 *    ensuring perfect closure in 3D (no visible gap).
 */
void ClosedSubdivisionCurve::laneRiesenfeldSubdivision()
{

  // Copy the original control points
  GMlib::DVector<GMlib::Vector<float, 3>> points = _controlPoints;

  // Perform Lane-Riesenfeld subdivision _degree_ times
  for (int iter = 0; iter < _degree; ++iter)
  {

    int numPoints = points.getDim();
    GMlib::DVector<GMlib::Vector<float, 3>> newPoints(2 * numPoints, GMlib::Vector<float, 3>(0.0f, 0.0f, 0.0f));

    // 1. Insert midpoints
    for (int i = 0; i < numPoints; ++i)
    {
      newPoints[2 * i] = points[i];
      int nxt = (i + 1) % numPoints; // wrap around
      newPoints[2 * i + 1] = (points[i] + points[nxt]) * 0.5f;
    }

    // 2. Perform averaging passes
    for (int avg = 1; avg < _degree; ++avg)
    {
      GMlib::DVector<GMlib::Vector<float, 3>> smoothedPoints(newPoints.getDim(), GMlib::Vector<float, 3>(0.0f, 0.0f, 0.0f));
      for (int i = 0; i < newPoints.getDim(); ++i)
      {
        int prev = (i - 1 + newPoints.getDim()) % newPoints.getDim();
        smoothedPoints[i] = (newPoints[i] + newPoints[prev]) * 0.5f;
      }
      newPoints = smoothedPoints;
    }

    points = newPoints; // Update points after each iteration
  }

  // Store final subdivided points
  _subdividedPoints = points;

  // Force the last point to match the first point, ensuring no gap
  // (only if we have at least 2 points)
  if (_subdividedPoints.getDim() > 1)
  {
    _subdividedPoints[_subdividedPoints.getDim() - 1] = _subdividedPoints[0];
  }
}

#endif // CLOSED_SUBDIVISION_CURVE_H
