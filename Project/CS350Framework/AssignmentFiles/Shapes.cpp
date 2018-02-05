///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

void PointExpansion(const std::vector<Vector3>& points, Vector3& center, float& radius)
{
  for (auto& pt : points) {
    Vector3 pmc = pt - center;
    float length = Math::Length(pmc);
    if (length > radius) {
      float r = 0.5f * (length + radius);
      center = center - pmc / length * (r - radius);
      radius = r;
    }
  }
}

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
  mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
  mStart = start;
  mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
  return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
  mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
  mStart = start;
  mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
  Ray transformedRay;
  transformedRay.mStart = Math::TransformPoint(transform, mStart);
  transformedRay.mDirection = Math::TransformDirection(transform, mDirection);
  return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
  return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
  return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
  Vector3 u;
  for (auto& pt : points) {
    u += pt;
  }
  u /= (float)points.size();

  Matrix3 c(0,0,0,0,0,0,0,0,0);
  for (auto& pt : points) {
    for (unsigned i = 0; i < 3; ++i) {
      for (unsigned j = 0; j < 3; ++j) {
        c[i][j] += (pt[i] - u[i]) * (pt[j] - u[j]);
      }
    }
  }
  c /= (float)points.size();
  return c;
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
  float apq = Math::NegativeMin();
  int p = -1;
  int q = -1;
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      if (i == j) {
        continue;
      }

      if (matrix[i][j] > apq) {
        apq = matrix[i][j];
        p = i;
        q = j;
      }
    }
  }

  float b = (matrix[q][q] - matrix[p][p]) / 2 * apq;
  float t = Math::GetSign(b) / (Math::Abs(b) + Math::Sqrt(b * b + 1));
  float c = Math::Sqrt(1 / (t * t + 1));
  float s = t * c;

  Matrix3 J = Matrix3::cIdentity;
  J[p][p] = J[q][q] = c;
  J[p][q] = s;
  J[q][p] = -s;
  return J;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations)
{
  Matrix3 diagonal = covariance;
  for (unsigned i = 0; i < maxIterations; ++i) {
    Matrix3 J = ComputeJacobiRotation(diagonal);
    eigenVectors = eigenVectors * J;
    diagonal = Math::Inverted(J) * diagonal * J;
    float offSome = 0.0f;
    for (unsigned j = 0; j < 3; ++j) {
      for (unsigned k = 0; k < 3; ++k) {
        if (j == k) {
          continue;
        }

        offSome += diagonal[i][j];
      }
    }

    if (Math::Abs(offSome) < 0.001f) {
      break;
    }
  }

  for (unsigned i = 0; i < 3; ++i) {
    eigenValues[i] = diagonal[i][i];
  }
}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
  mCenter = Vector3::cZero;
  mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
  mCenter = center;
  mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
  if (points.empty()) {
    return;
  }
  mCenter = Vector3();
  for (auto& pt : points) {
    mCenter += pt;
  }
  mCenter /= (float)points.size();

  mRadius = 0.0f;
  for (auto& pt : points) {
    mRadius = Math::Max(mRadius, Math::Length(pt - mCenter));
  }
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
  if (points.empty()) {
    return;
  }

  Vector3 minAxes[3] = { Vector3(Math::PositiveMax()) };
  Vector3 maxAxes[3] = { Vector3(Math::NegativeMin()) };

  for (auto& pt : points) {
    // check if the current point is the max or min of any of the three axes
    for (unsigned i = 0; i < 3; ++i) {
      if (minAxes[i][i] != Math::Min(minAxes[i][i], pt[i])) {
        minAxes[i] = pt;
      }
      if (maxAxes[i][i] != Math::Max(maxAxes[i][i], pt[i])) {
        maxAxes[i] = pt;
      }
    }
  }

  int largestSpreadAxis = -1;
  float largestLength = 0;
  for (unsigned i = 0; i < 3; ++i) {
    float length = Math::Length(maxAxes[i] - minAxes[i]);
    if (length > largestLength) {
      largestLength = length;
      largestSpreadAxis = i;
    }
  }

  mCenter = (maxAxes[largestSpreadAxis] + minAxes[largestSpreadAxis]) * 0.5f;
  mRadius = largestLength * 0.5f;

  PointExpansion(points, mCenter, mRadius);
}

void Sphere::ComputePCA(const std::vector<Vector3>& points, int maxIterations)
{
  // The PCA method:
  // Compute the eigen values and vectors. Take the largest eigen vector as the axis of largest spread.
  // Compute the sphere center as the center of this axis then expand by all points.
  Vector3 eigenValues;
  Matrix3 eigenVectors;
  ComputeEigenValuesAndVectors(ComputeCovarianceMatrix(points), eigenValues, eigenVectors, maxIterations);

  int largestAxis = -1;
  float largestLength = Math::NegativeMin();
  for (unsigned i = 0; i < 3; ++i) {
    float length = Math::Length(eigenVectors[i]);
    if (length > largestLength) {
      largestLength = length;
      largestAxis = i;
    }
  }

  Vector3 maxPoint = Vector3(Math::NegativeMin());
  Vector3 minPoint = Vector3(Math::PositiveMax());

  for (auto& pt : points) {
    Vector3 pt2 = Math::Transform(eigenVectors, pt);

    if (pt2[largestAxis] < minPoint[largestAxis]) {
      minPoint = pt2;
    }
    if (pt2[largestAxis] > maxPoint[largestAxis]) {
      maxPoint = pt2;
    }
  }

  Matrix3 inverse = Math::Inverted(eigenVectors);
  maxPoint = Math::Transform(inverse, maxPoint);
  minPoint = Math::Transform(inverse, minPoint);

  mCenter = (maxPoint + minPoint) * 0.5f;
  mRadius = Math::Length(maxPoint - mCenter);

  PointExpansion(points, mCenter, mRadius);
}

bool Sphere::ContainsPoint(const Vector3& point)
{
  return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
  return mCenter;
}

float Sphere::GetRadius() const
{
  return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
  float posDiff = Math::Length(mCenter - rhs.mCenter);
  float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

  return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
  return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
  //set the aabb to an initial bad value (where the min is smaller than the max)
  mMin.Splat(Math::PositiveMax());
  mMax.Splat(Math::NegativeMin());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
  mMin = min;
  mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
  return Aabb(center - halfExtents, center + halfExtents);
}

Aabb Aabb::BuildFromMinMax(const Vector3& min, const Vector3& max)
{
  return Aabb(min, max);
}

float Aabb::GetVolume() const
{
  Vector3 dim = mMax - mMin;
  return dim[0] * dim[1] * dim[2];
}

float Aabb::GetSurfaceArea() const
{
  Vector3 dim = mMax - mMin;
  return 2 * dim[0] * dim[1] + 2 * dim[0] * dim[2] + 2 * dim[1] * dim[2];
}

void Aabb::Compute(const std::vector<Vector3>& points)
{
  mMin.Splat(Math::PositiveMax());
  mMax.Splat(Math::NegativeMin());
  for(size_t i = 0; i < points.size(); ++i)
  {
    const Vector3& point = points[i];
    mMin = Math::Min(mMin, point);
    mMax = Math::Max(mMax, point);
  }
}

bool Aabb::Contains(const Aabb& aabb) const
{
  for (unsigned i = 0; i < 3; ++i) {
    if (aabb.GetMax()[i] > mMax[i] || aabb.GetMin()[i] < mMin[i]) {
      return false;
    }
  }
  return true;
}

void Aabb::Expand(const Vector3& point)
{
  for(size_t i = 0; i < 3; ++i)
  {
    mMin[i] = Math::Min(mMin[i], point[i]);
    mMax[i] = Math::Max(mMax[i], point[i]);
  }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
  Aabb result;
  for(size_t i = 0; i < 3; ++i)
  {
    result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
    result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
  }
  return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
  float pos1Diff = Math::Length(mMin - rhs.mMin);
  float pos2Diff = Math::Length(mMax - rhs.mMax);

  return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Matrix4& transform)
{
  /******Student:Assignment2******/
  // Compute aabb of the this aabb after it is transformed.
  // You should use the optimize method discussed in class (not transforming all 8 points).
  Warn("Assignment2: Required function un-implemented");

}

Vector3 Aabb::GetMin() const
{
  return mMin;
}

Vector3 Aabb::GetMax() const
{
  return mMax;
}

Vector3 Aabb::GetCenter() const
{
  return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
  return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
  return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
  mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  mPoints[0] = p0;
  mPoints[1] = p1;
  mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
  return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
  mData = Vector4::cZero;
}

Plane::Plane(const Triangle& triangle)
{
  Set(triangle);
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
  Set(normal, point);
}

void Plane::Set(const Triangle& triangle)
{
  // Set from the triangle's points
  Set(triangle.mPoints[0], triangle.mPoints[1], triangle.mPoints[2]);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Vector3 normal = Math::Cross(p1 - p0, p2 - p0);
  normal.AttemptNormalize();
  mData = Vector4(normal.x, normal.y, normal.z, Math::Dot(normal, p0));
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
  Vector3 norm = normal;
  norm.AttemptNormalize();
  mData = Vector4(norm.x, norm.y, norm.z, Math::Dot(norm, point));
}

Vector3 Plane::GetNormal() const
{
  return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
  return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
  return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
  return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
                  const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
  mPoints[0] = lbn;
  mPoints[1] = rbn;
  mPoints[2] = rtn;
  mPoints[3] = ltn;
  mPoints[4] = lbf;
  mPoints[5] = rbf;
  mPoints[6] = rtf;
  mPoints[7] = ltf;

  //left
  mPlanes[0].Set(lbf, ltf, lbn);
  //right
  mPlanes[1].Set(rbn, rtf, rbf);
  //top
  mPlanes[2].Set(ltn, ltf, rtn);
  //bot
  mPlanes[3].Set(rbn, lbf, lbn);
  //near
  mPlanes[4].Set(lbn, ltn, rbn);
  //far
  mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
  return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
  return gDebugDrawer->DrawFrustum(*this);
}
