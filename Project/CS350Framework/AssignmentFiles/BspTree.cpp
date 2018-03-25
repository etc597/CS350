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
  if (!coplanarFront.empty())
  {
    triangles.insert(triangles.end(), coplanarFront.begin(), coplanarFront.end());
  }

  if (!coplanarBack.empty())
  {
    triangles.insert(triangles.end(), coplanarBack.begin(), coplanarBack.end());
  }
}

void BspTreeNode::ClipTo(BspTreeNode* node, float epsilon)
{
  if (!node) return;

  node->ClipTriangles(coplanarFront, epsilon);
  node->ClipTriangles(coplanarBack, epsilon);

  if (front) front->ClipTo(node, epsilon);
  if (back) back->ClipTo(node, epsilon);
}

void BspTreeNode::ClipTriangles(TriangleList& triangles, float epsilon)
{
  TriangleList result;
  for (auto& tri : triangles)
  {
    ClipTriangle(tri, result, epsilon);
  }
  triangles = result;
}

void BspTreeNode::ClipTriangle(Triangle& triangle, TriangleList& results, float epsilon)
{
  TriangleList rFront, rBack; // since we want them grouped anyways, just use 2 lists
  BspTree::SplitTriangle(splitPlane, triangle, rFront, rBack, rFront, rBack, epsilon);

  // if on the back side of a plane that has no back children -> solid leaf
  if (back)
  {
    for (auto& tri : rFront)
    {
      back->ClipTriangle(tri, results, epsilon);
    }
  }
  else
  {
    // no - op, don't add the back triangles to the results
  }

  if (front)
  {
    for (auto& tri : rFront)
    {
      front->ClipTriangle(tri, results, epsilon);
    }
  }
  else
  {
    results.insert(results.end(), rFront.begin(), rFront.end());
  }
}

bool InFront(IntersectionType::Type type)
{
  return type == IntersectionType::Inside;
}

bool Behind(IntersectionType::Type type)
{
  return type == IntersectionType::Outside;
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
  if (points.size() == 4)
  {
    auto tri2 = Triangle(points[0], points[2], points[3]);
    PlaneTriangle(plane.mData, tri2.mPoints[0], tri2.mPoints[1], tri2.mPoints[2], epsilon) == IntersectionType::Coplanar
      ? coplanarList.push_back(tri2)
      : triList.push_back(tri2);
  }
}


//--------------------------------------------------------------------BspTree
BspTree::BspTree()
  : mRoot(nullptr)
{
}

BspTree::~BspTree()
{
  Destruct();
}

void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
  auto intersection = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);
  
  switch (intersection)
  {
  case IntersectionType::Inside:
    front.push_back(tri);
    break;
  case IntersectionType::Outside:
    back.push_back(tri);
    break;
  case IntersectionType::Coplanar:
  {
    float dot = Math::Dot(plane.GetNormal(), Plane(tri).GetNormal());
    dot >= 0 ? coplanarFront.push_back(tri) : coplanarBack.push_back(tri);
    break;
  }
  case IntersectionType::Overlaps:
  {
    std::vector<Point> frontPoints;
    std::vector<Point> backPoints;
    for (unsigned i = 0; i < 3; ++i)
    {
      auto A = tri.mPoints[i];
      auto B = tri.mPoints[(i + 1) % 3];

      ClippingTable(A, B, plane, frontPoints, backPoints, epsilon);
    }
    ConstructTrianglesForList(frontPoints, plane, front, coplanarFront, epsilon);
    ConstructTrianglesForList(backPoints, plane, back, coplanarBack, epsilon);
  }
    break;
  default:
    break;
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
      break;
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
  if (mRoot)
  {
    Destruct();
  }

  // only return after destroying - empty triangle list implies empty tree
  if (triangles.empty())
  {
    return;
  }

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

  float tMin = 0.0f, tMax = Math::PositiveMax();

  RayCast(mRoot, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  return t != Math::PositiveMax();
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
  std::stack<Node*> toGet;
  if (mRoot) toGet.push(mRoot);
  while (!toGet.empty())
  {
    Node* node = toGet.top();
    toGet.pop();
    if (node->front) toGet.push(node->front);
    if (node->back) toGet.push(node->back);
    node->GetTriangles(triangles);
  }
}

void BspTree::Invert()
{
  std::stack<Node*> toInvert;
  if (mRoot) toInvert.push(mRoot);
  while (!toInvert.empty())
  {
    Node* node = toInvert.top();
    toInvert.pop();
    if (node->front) toInvert.push(node->front);
    if (node->back) toInvert.push(node->back);

    node->splitPlane.mData *= -1;

    TriangleList triangles;
    node->GetTriangles(triangles);

    for (auto& tri : triangles)
    {
      std::swap(tri.mPoints[0], tri.mPoints[1]);
    }

    std::swap(node->front, node->back);
  }
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
  mRoot->ClipTo(tree->mRoot, epsilon);
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
  mRoot->ClipTo(tree->mRoot, epsilon);
  tree->mRoot->ClipTo(mRoot, epsilon);

  // Remove Coplanar faces
  tree->Invert();
  tree->mRoot->ClipTo(mRoot, epsilon);
  tree->Invert();

  TriangleList results;
  AllTriangles(results);
  tree->AllTriangles(results);
  Construct(results, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
  Invert();
  tree->Invert();
  Union(tree, k, epsilon);
  Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
  tree->Invert();
  Intersection(tree, k, epsilon);
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
  std::stack<std::pair<Node*, int>> nodes;
  if (mRoot) nodes.emplace(mRoot, 0);
  while (!nodes.empty())
  {
    auto pair = nodes.top();
    nodes.pop();
    Node* top = pair.first;
    int lvl = pair.second;
    if (level == -1 || lvl < level)
    {
      if(top->front) nodes.emplace(top->front, lvl + 1);
      if(top->back) nodes.emplace(top->back, lvl + 1);
    }

    if (level == -1 || level == lvl)
    {
      TriangleList triangles;
      top->GetTriangles(triangles);
      for (auto& tri : triangles)
      {
        tri.DebugDraw().Color(color).SetMaskBit(bitMask);
      }
    }
  }
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

void BspTree::Destruct()
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
  mRoot = nullptr;
}

void BspTree::RayCast(Node * node, const Ray & ray, float & t, float tMin, float tMax, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  if (node == nullptr)
  {
    return;
  }

  float tPlane;
  Plane& plane = node->splitPlane;
  auto intersection = PointPlane(ray.mStart, plane.mData, planeThicknessEpsilon);
  Node* nearSide;
  Node* farSide;

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

  // edge case 1
  if (Coplanar(intersection))
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    RayCast(farSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    // check plane geometry
    TriangleList tris;
    node->GetTriangles(tris);
    for (auto& tri : tris)
    {
      float triangleT = 0.0f;
      if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], triangleT, triExpansionEpsilon))
      {
        t = std::min(triangleT, t);
      }
    }

    return;
  }
  
  auto res = RayPlane(ray.mStart, ray.mDirection, plane.mData, tPlane);

  // edge case 2
  if (res == false)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    return;
  }

  float te = Math::Abs(planeThicknessEpsilon / Math::Dot(plane.GetNormal(), ray.mDirection));

  // case 1
  if (tMin - te <= tPlane && tPlane <= tMax + te)
  {
    RayCast(nearSide, ray, t, tMin, tPlane + te, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    RayCast(farSide, ray, t, tPlane - te, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    // check plane triangles
    TriangleList tris;
    node->GetTriangles(tris);
    for (auto& tri : tris)
    {
      float triangleT = 0.0f;
      if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], triangleT, triExpansionEpsilon))
      {
        t = std::min(triangleT, t);
      }
    }
  }
  // case 2
  else if (tPlane < 0.0f)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
  }
  // case 3
  else if (tMax < tPlane)
  {
    RayCast(nearSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

  }
  // case 4
  else if (0 < tPlane && tPlane < tMin)
  {
    RayCast(farSide, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
  }
}

