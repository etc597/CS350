///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//--------------------------------------------------------------------BspTreeNode
BspTreeNode* BspTreeNode::GetFrontChild() const
{
  return front;
}

BspTreeNode* BspTreeNode::GetBackChild() const
{
  return back;
}

Plane BspTreeNode::GetSplitPlane() const
{
  return splitPlane;
}

void BspTreeNode::GetTriangles(TriangleList& triangles) const
{
  triangles = coplanarFront;
  triangles.insert(triangles.end(), coplanarBack.begin(), coplanarBack.end());
}

bool InFront(IntersectionType::Type type)
{
  return type == IntersectionType::Outside;
}

bool Behind(IntersectionType::Type type)
{
  return type == IntersectionType::Inside;
}

bool Coplanar(IntersectionType::Type type)
{
  return type == IntersectionType::Coplanar;
}

typedef Math::Vector3 Point;

void ClippingTable(const Point& A, const Point& B, const Plane& plane, std::vector<Point>& front, std::vector<Point>& back, float epsilon)
{
  auto a = PointPlane(A, plane.mData, epsilon);
  auto b = PointPlane(B, plane.mData, epsilon);

  if (InFront(a) && InFront(b))
  {
    front.push_back(B);
  }
  else if (Coplanar(a) && InFront(b))
  {
    front.push_back(B);
  }
  else if (Behind(a) && InFront(b))
  {
    Point I;
    float t;
    auto res = RayPlane(A, B - A, plane.mData, t);
    if (res && t <= 1.0f)
    {
      I = A + t * (B - A);
      front.push_back(I);
      back.push_back(I);
    }
    front.push_back(B);
  }
  else if (InFront(a) && Coplanar(b))
  {
    front.push_back(B);

  }
  else if (Coplanar(a) && Coplanar(b))
  {
    front.push_back(B);

  }
  else if (Behind(a) && Coplanar(b))
  {
    front.push_back(B);
    back.push_back(B);
  }
  else if (InFront(a) && Behind(b))
  {
    Point I;
    float t;
    auto res = RayPlane(A, B - A, plane.mData, t);
    if (res && t <= 1.0f)
    {
      I = A + t * (B - A);
      front.push_back(I);
      back.push_back(I);
    }
    back.push_back(B);
  }
  else if (Coplanar(a) && Behind(b))
  {
    back.push_back(A);
    back.push_back(B);
  }
  else if (Behind(a) && Behind(b))
  {
    back.push_back(B);
  }
}

void ConstructTrianglesForList(const std::vector<Point>& points, const Plane& plane, TriangleList& triList, TriangleList& coplanarList, float epsilon)
{
  auto tri1 = Triangle(points[0], points[1], points[2]);
  
  PlaneTriangle(plane.mData, tri1.mPoints[0], tri1.mPoints[1], tri1.mPoints[2], epsilon) == IntersectionType::Coplanar
    ? coplanarList.push_back(tri1)
    : triList.push_back(tri1);


  // if we have a quad
  if (points.size == 4)
  {
    auto tri2 = Triangle(points[0], points[2], points[3]);
    PlaneTriangle(plane.mData, tri2.mPoints[0], tri2.mPoints[1], tri2.mPoints[2], epsilon) == IntersectionType::Coplanar
      ? coplanarList.push_back(tri2)
      : triList.push_back(tri2);
  }
}


//--------------------------------------------------------------------BspTree
BspTree::BspTree()
{
}

BspTree::~BspTree()
{
  std::stack<Node*> toDel;
  if (mRoot) toDel.push(mRoot);
  while (!toDel.empty())
  {
    Node* node = toDel.top();
    toDel.pop();
    if (node->front) toDel.push(node->front);
    if (node->back) toDel.push(node->back);
    delete node;
  }
}

void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
  auto intersection = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);
  
  switch (intersection)
  {
  case IntersectionType::Outside:
    front.push_back(tri);
    break;
  case IntersectionType::Inside:
    back.push_back(tri);
    break;
  case IntersectionType::Coplanar:
    float dot = Math::Dot(plane.GetNormal(), Plane(tri).GetNormal());
    dot >= 0 ? coplanarFront.push_back(tri) : coplanarBack.push_back(tri);
    break;
  case IntersectionType::Overlaps:
  {
    std::vector<Point> frontPoints;
    std::vector<Point> backPoints;
    for (unsigned i = 0; i < 3; ++i)
    {
      auto A = tri.mPoints[i - 1 % 3];
      auto B = tri.mPoints[i];

      ClippingTable(A, B, plane, frontPoints, backPoints, epsilon);
    }
    ConstructTrianglesForList(frontPoints, plane, front, coplanarFront, epsilon);
    ConstructTrianglesForList(backPoints, plane, back, coplanarBack, epsilon);
  }
    break;
  default:
  }

}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
  auto testPlane = Plane(triangles[testIndex]);

  if (Math::Dot(testPlane.GetNormal(), testPlane.GetNormal()) < epsilon * epsilon)
  {
    return Math::PositiveMax();
  }

  int ns = 0, nf = 0, nb = 0;
  for (auto& tri : triangles)
  {
    auto intersection = PlaneTriangle(testPlane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);
    switch (intersection)
    {
    case IntersectionType::Outside:
      ++nf;
      break;
    case IntersectionType::Inside:
      ++nb;
      break;
    case IntersectionType::Overlaps:
      ++ns;
      break;
    default:
    }
  }

  return k * ns + (1 - k) * Math::Abs(nf - nb);
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
  float bestScore = Math::PositiveMax();
  size_t bestIndex = -1;
  for (size_t i = 0; i < triangles.size(); ++i)
  {
    auto score = CalculateScore(triangles, i, k, epsilon);
    if (score < bestScore) 
    {
      bestScore = score;
      bestIndex = i;
    }
  }
  
  return bestIndex;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
  size_t index = PickSplitPlane(triangles, k, epsilon);
  mRoot = new Node;
  Node& node = *mRoot;
  node.splitPlane = Plane(triangles[index]);

  TriangleList front, back;
  for (auto& tri : triangles)
  {
    SplitTriangle(node.splitPlane, tri, node.coplanarFront, node.coplanarBack, front, back, epsilon);
  }

  if (!front.empty())
  {
    Construct(node.front, front, k, epsilon);
  }
  if (!back.empty())
  {
    Construct(node.back, back, k, epsilon);
  }
}

BspTreeNode* BspTree::GetRoot() const
{
  return mRoot;
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  t = Math::PositiveMax();
  if (!mRoot)
  {
    return false;
  }

  float tMin = 0.0f, tMax = Math::PositiveMax();

  auto res = RayCast(mRoot, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  return res;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::Invert()
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
  /******Student:Assignment4******/
  Warn("Assignment4: Required function un-implemented");
}

void BspTree::Construct(Node *& newNode, const TriangleList & triangles, float k, float epsilon)
{
  newNode = new Node;
  Node& node = *newNode;

  size_t index = triangles.size() == 1 ? 0 : PickSplitPlane(triangles, k, epsilon);
  node.splitPlane = Plane(triangles[index]);

  TriangleList front, back;
  for (auto& tri : triangles)
  {
    SplitTriangle(node.splitPlane, tri, node.coplanarFront, node.coplanarBack, front, back, epsilon);
  }

  if (!front.empty())
  {
    Construct(node.front, front, k, epsilon);
  }
  if (!back.empty())
  {
    Construct(node.back, back, k, epsilon);
  }
}

bool BspTree::RayCast(Node * node, const Ray & ray, float & t, float tMin, float tMax, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  float tPlane;
  Plane& plane = node->splitPlane;
  auto intersection = PointPlane(ray.mStart, plane.mData, planeThicknessEpsilon);
  Node* nearSide;
  Node* farSide;

  auto res = RayPlane(ray.mStart, ray.mDirection, plane.mData, tPlane);

  if (InFront(intersection))
  {
    nearSide = node->front;
    farSide = node->back;
  }
  else
  {
    farSide = node->front;
    nearSide = node->back;
  }

  float te = Math::Abs(planeThicknessEpsilon / Math::Dot(plane.GetNormal(), ray.mDirection));

  if (Coplanar(intersection))
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    RayCast(farSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    // check plane geometry
    // don't update
  }
  else if (res == false)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
  }
  else if (tMin - te <= tPlane && tPlane <= tMax + te)
  {
    RayCast(nearSide, ray, t, tMin, tPlane, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    RayCast(farSide, ray, t, tPlane, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    // check plane triangles
  }
  else if (tPlane < 0.0f)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  }
  else if (tMax < tPlane)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  }
  else if (0 < tPlane && tPlane < tMin)
  {
    RayCast(farSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  }
}

