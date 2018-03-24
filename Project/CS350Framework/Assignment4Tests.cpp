#include "Precompiled.hpp"
#include "Application.hpp"
#include "Shapes.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "DebugDraw.hpp"
#include "Components.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"
#include "DynamicAabbTree.hpp"
#include "BspTree.hpp"

float splitWeight = 0.8f;
float epsilon = 0.0001f;
float triExpansionEpsilon = 0.0001f;

std::string GetApplicationDirectory()
{
  char temp[MAX_PATH + 1];
  GetModuleFileName(nullptr, temp, MAX_PATH);
  std::string fileName = std::string(temp);

  size_t index = fileName.find_last_of('\\');
  if(index == std::string::npos)
    return std::string();

  std::string applicationDir = fileName.substr(0, index);
  return applicationDir;
}

void LoadMesh(const std::string& fileName, const Matrix4& transform, TriangleList& results)
{
  std::string appDir = GetApplicationDirectory();
  std::string filePath = appDir + "\\" + fileName;

  const size_t bufferSize = 1000;
  char buffer[bufferSize];
  size_t size;

  std::string fileData;
  FILE* file;
  fopen_s(&file, filePath.c_str(), "r");
  if(file == nullptr)
    return;

  do
  {
    size = fread_s(buffer, bufferSize - 1, 1, bufferSize - 1, file);
    buffer[size] = NULL;
    fileData += buffer;
  } while(size != 0);
  fclose(file);

  std::vector<Vector3> vertices;
  std::vector<size_t> indices;
  Helpers::LoadObjFile(fileData, vertices, indices);
  
  // Add a scale to the mesh to help reduce numerical issues
  float scale = 100.0f;
  for(size_t i = 0; i < indices.size(); i += 3)
  {
    Vector3 p0 = Math::TransformPoint(transform, vertices[indices[i + 0]]) * scale;
    Vector3 p1 = Math::TransformPoint(transform, vertices[indices[i + 1]]) * scale;
    Vector3 p2 = Math::TransformPoint(transform, vertices[indices[i + 2]]) * scale;
    results.push_back(Triangle(p0, p1, p2));
  }
}

void PrintTriangleList(const std::string listName, const TriangleList& triangles, FILE* file)
{
  if(file == nullptr)
    return;

  // sort the triangles to help guarantee a consistent order
  TriangleList sortedTriangles = triangles;
  std::sort(sortedTriangles.begin(), sortedTriangles.end());

  fprintf(file, "  %s:\n", listName.c_str());
  if(sortedTriangles.empty())
  {
    fprintf(file, "    Empty\n");
  }
  else
  {
    for(size_t i = 0; i < sortedTriangles.size(); ++i)
      fprintf(file, "    %s\n", PrintTriangle(sortedTriangles[i]).c_str());
  }
}

void SimplifyAndDrawTriangles(TriangleList& tris)
{
  std::vector<Helpers::SimpleTriangle> results;
  for(size_t i = 0; i < tris.size(); ++i)
  {
    Helpers::SimpleTriangle tri;
    tri.mPoints[0] = tris[i].mPoints[0];
    tri.mPoints[1] = tris[i].mPoints[1];
    tri.mPoints[2] = tris[i].mPoints[2];
    results.push_back(tri);
  }

  std::vector<Helpers::SimplePolygon> mergedResults;
  Simplify(results, mergedResults);

  for(size_t i = 0; i < mergedResults.size(); ++i)
  {
    Helpers::SimplePolygon& poly = mergedResults[i];
    for(size_t j = 0; j < poly.mPoints.size(); ++j)
    {
      Vector3 p0 = poly.mPoints[j];
      Vector3 p1 = poly.mPoints[(j + 1) % poly.mPoints.size()];
      gDebugDrawer->DrawLine(LineSegment(p0, p1));
    }
  }
}

void CreateShape1(TriangleList& results)
{
  results.push_back(Triangle(Vector3(0, 3, 0), Vector3(1, 5, 0), Vector3(1, 5, 1)));
  results.push_back(Triangle(Vector3(1, 5, 1), Vector3(0, 3, 0), Vector3(0, 3, 1)));

  results.push_back(Triangle(Vector3(1, 5, 0), Vector3(3, 5, 0), Vector3(3, 5, 1)));
  results.push_back(Triangle(Vector3(3, 5, 1), Vector3(1, 5, 0), Vector3(1, 5, 1)));
  
  results.push_back(Triangle(Vector3(3, 5, 0), Vector3(4, 4, 0), Vector3(4, 4, 1)));
  results.push_back(Triangle(Vector3(4, 4, 1), Vector3(3, 5, 0), Vector3(3, 5, 1)));
  
  results.push_back(Triangle(Vector3(4, 4, 0), Vector3(4, 3, 0), Vector3(4, 3, 1)));
  results.push_back(Triangle(Vector3(4, 3, 1), Vector3(4, 4, 0), Vector3(4, 4, 1)));
  
  results.push_back(Triangle(Vector3(4, 3, 0), Vector3(2, 2, 0), Vector3(2, 2, 1)));
  results.push_back(Triangle(Vector3(2, 2, 1), Vector3(4, 3, 0), Vector3(4, 3, 1)));
  
  results.push_back(Triangle(Vector3(2, 2, 0), Vector3(2, 0, 0), Vector3(2, 0, 1)));
  results.push_back(Triangle(Vector3(2, 0, 1), Vector3(2, 2, 0), Vector3(2, 2, 1)));
  
  results.push_back(Triangle(Vector3(2, 0, 0), Vector3(0, 3, 0), Vector3(0, 3, 1)));
  results.push_back(Triangle(Vector3(0, 3, 1), Vector3(2, 0, 0), Vector3(2, 0, 1)));
}

void CreateShape2(TriangleList& results)
{
  results.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  results.push_back(Triangle(Vector3(0, 1, 1), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  results.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  results.push_back(Triangle(Vector3(1, 2, 1), Vector3(0, 1, 0), Vector3(0, 1, 1)));

  results.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  results.push_back(Triangle(Vector3(3, 2, 1), Vector3(1, 2, 0), Vector3(1, 2, 1)));

  results.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  results.push_back(Triangle(Vector3(4, 1, 1), Vector3(3, 2, 0), Vector3(3, 2, 1)));

  results.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  results.push_back(Triangle(Vector3(3, 0, 1), Vector3(4, 1, 0), Vector3(4, 1, 1)));

  results.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
  results.push_back(Triangle(Vector3(2, 1.5f, 1), Vector3(3, 0, 0), Vector3(3, 0, 1)));

  results.push_back(Triangle(Vector3(2, 1.5f, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));
  results.push_back(Triangle(Vector3(1, 0, 1), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
}

// Just a small helper to shrink a triangle (so it can be more easily viewed after a split)
Triangle ShrinkTriangle(const Triangle& tri, float scaleFactor)
{
  Vector3 center = (tri.mPoints[0] + tri.mPoints[1] + tri.mPoints[2]) / 3.0f;
  Triangle result;
  for(size_t i = 0; i < 3; ++i)
    result.mPoints[i] = tri.mPoints[i] - scaleFactor * Math::Normalized(tri.mPoints[i] - center);
  return result;
}

void TestSplitPick(const std::string& testName, const TriangleList& triangles, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  BspTree bspTree;
  std::vector<float> scores;
  float epsilon = 0.0001f;
  int splitTriangleIndex = bspTree.PickSplitPlane(triangles, splitWeight, epsilon);
  for(size_t i = 0; i < triangles.size(); ++i)
  {
    float score = BspTree::CalculateScore(triangles, i, splitWeight, epsilon);
    scores.push_back(score);
  }

  for(size_t i = 0; i < triangles.size(); ++i)
  {
    DebugShape& shape = triangles[i].DebugDraw();
    if(i == splitTriangleIndex)
      shape.Color(Vector4(0, 0, 1, 1));
  }

  if(file != nullptr)
  {
    fprintf(file, "    Bsp Triangle Split Index: %d\n", splitTriangleIndex);
    fprintf(file, "    Scores: ");
    for(size_t i = 0; i < scores.size(); ++i)
      fprintf(file, "%s, ", PrintFloat(scores[i]).c_str());
    fprintf(file, "\n");
  }
}

void TestTriangleSplit(const std::string& testName, Plane& plane, Triangle& testTri, float epsilon, FILE* file)
{
  PrintTestHeader(file, testName);

  TriangleList coplanarFront, coplanarBack, front, back;
  BspTree::SplitTriangle(plane, testTri, coplanarFront, coplanarBack, front, back, epsilon);

  testTri.DebugDraw();
  plane.DebugDraw(5.0f);

  float scaleFactor = 0.05f;

  PrintTriangleList("Front", front, file);
  PrintTriangleList("Back", back, file);
  PrintTriangleList("CoplanarFront", coplanarFront, file);
  PrintTriangleList("CoplanarBack", coplanarBack, file);

  for(size_t i = 0; i < coplanarFront.size(); ++i)
    ShrinkTriangle(coplanarFront[i], scaleFactor).DebugDraw().Color(Vector4(0.5f));
  for(size_t i = 0; i < coplanarBack.size(); ++i)
    ShrinkTriangle(coplanarBack[i], scaleFactor).DebugDraw().Color(Vector4(0.5f));
  for(size_t i = 0; i < front.size(); ++i)
    ShrinkTriangle(front[i], scaleFactor).DebugDraw().Color(Vector4(1, 0, 0, 1));
  for(size_t i = 0; i < back.size(); ++i)
    ShrinkTriangle(back[i], scaleFactor).DebugDraw().Color(Vector4(0, 0, 1, 1));
}

void TestBspTreeStructure(const std::string& testName, TriangleList& triangles, FILE* file = nullptr)
{
  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  PrintTestHeader(file, testName);

  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  PrintSpatialPartitionStructure(bspTree, file);
}

void TestRay(BspTree& tree, Ray& ray, float epsilon, FILE* file)
{
  // Clear the old statistics
  Application::mStatistics.Clear();
  int debuggingIndex = Application::mUnitTestValues.mDebuggingIndex;

  float t = Math::PositiveMax();
  bool result = tree.RayCast(ray, t, epsilon, triExpansionEpsilon, debuggingIndex);

  // Record how many ray-plane and ray-triangle tests were performed
  size_t rayPlaneCount = Application::mStatistics.mRayPlaneTests;
  size_t rayTriangleCount = Application::mStatistics.mRayTriangleTests;

  if(!result)
    t = 100;
  ray.DebugDraw(t);

  if(file != nullptr)
  {
    if(result)
      fprintf(file, "  t: %s\n", PrintFloat(t).c_str());
    else
      fprintf(file, "  false\n");
    fprintf(file, "  Statistics: RayPlane(%d) RayTri(%d)\n", rayPlaneCount, rayTriangleCount);
  }
}

void TestRays(BspTree& tree, std::vector<Ray>& rays, float epsilon, FILE* file)
{
  PrintSpatialPartitionStructure(tree, file, "Tree Structure");

  for(size_t i = 0; i < rays.size(); ++i)
    TestRay(tree, rays[i], epsilon, file);
}

void TestRaycasts(BspTree& bspTree, Vector3& rayStart, float subDivisions, FILE* file)
{
  float deltaRad = (Math::cTwoPi / subDivisions);
  for(float rad = 0; rad <= Math::cTwoPi; rad += deltaRad)
  {
    Vector3 dir = Vector3(Math::Cos(rad), Math::Sin(rad), 0);
    Ray ray(rayStart, dir);

    TestRay(bspTree, ray, epsilon * 2, file);
  }
}

void TestRaycasts(TriangleList& triangles, std::vector<Vector3>& rayStarts, float subDivisions, FILE* file)
{
  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  PrintSpatialPartitionStructure(bspTree, file, "Tree Structure");

  for(size_t i = 0; i < rayStarts.size(); ++i)
    TestRaycasts(bspTree, rayStarts[i], subDivisions, file);
}

void TestRaycastsSpherical(const std::string testName, TriangleList& triangles, Vector3& rayStart, float stacks, float slices, FILE* file)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  PrintSpatialPartitionStructure(bspTree, file, "Tree Structure");

  for(size_t iStack = 0; iStack < stacks; ++iStack)
  {
    for(size_t iSlice = 0; iSlice < slices; ++iSlice)
    {
      float theta = Math::cTwoPi * (iSlice / (float)slices);
      float phi = Math::cPi * ((iStack + 0.5f)/ (float)stacks);

      float x = Math::Sin(theta) * Math::Sin(phi);
      float y = Math::Cos(phi);
      float z = Math::Cos(theta) * Math::Sin(phi);

      Ray ray;
      ray.mStart = rayStart;
      ray.mDirection = Vector3(x, y, z);
      TestRay(bspTree, ray, epsilon * 2, file);
    }
  }
}

void TriangleSplitTest1(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest2(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest3(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest4(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest5(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest6(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest7(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest8(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest9(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));
  
  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest10(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest11(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest12(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest13(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest14(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest15(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon,  file);
}

void TriangleSplitTest16(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest17(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest18(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest19(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest20(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest21(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest22(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest23(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest24(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest25(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest26(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest27(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest28(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest29(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(2, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));
  
  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest30(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(2, 0, 0), Vector3(2, 1, 1), Vector3(2, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest31(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-2, 0, 0), Vector3(2, 4, 0), Vector3(0, 2, 2));
  Plane plane;
  plane.Set(Vector3(1, -1, 0).Normalized(), Vector3(0, 2, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest32(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-2, 0, 0), Vector3(0, 2, 2), Vector3(2, 4, 0));
  Plane plane;
  plane.Set(Vector3(1, -1, 0).Normalized(), Vector3(0, 2, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void TriangleSplitTest33(const std::string& testName, FILE* file = nullptr)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(1, 0, -1), Vector3(-1, 0, 0), Vector3(-1, 0, 1));
  Plane plane;
  plane.mData = Vector4(0, 0, -1, 0);

  TestTriangleSplit(testName, plane, tri, epsilon, file);
}

void SplitPlanePickTest1(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  for(float i = 0; i < 9.0f; i += 1.0f)
    triangles.push_back(Triangle(Vector3(i, 0, 0), Vector3(i, 0, 1), Vector3(i, 1, 0)));

  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest2(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  for(float i = 0; i < 6.0f; i += 1.0f)
    triangles.push_back(Triangle(Vector3(i, 0, 0), Vector3(i, 0, 1), Vector3(i, 1, 0)));
  triangles.push_back(Triangle(Vector3(2.5f, 2, 0), Vector3(3.5f, 2, 0), Vector3(3, 2, 2)));
  triangles.push_back(Triangle(Vector3(2.5f, -2, 0), Vector3(3.5f, -2, 0), Vector3(3, -2, 2)));

  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest3(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, 0, 1), Vector3(1, 1, 0)));
  triangles.push_back(Triangle(Vector3(4, 0, 0), Vector3(4, 0, 1), Vector3(4, 1, 0)));
  triangles.push_back(Triangle(Vector3(2.75f, 4, 0), Vector3(2.75f, 4, 1), Vector3(2.75f, 5, 0)));
  triangles.push_back(Triangle(Vector3(3.25f,-4, 0), Vector3(3.25f,-4, 1), Vector3(3.25f,-5, 0)));

  triangles.push_back(Triangle(Vector3(2.5f, 0, 0), Vector3(2.5f, 0, 1), Vector3(3.5f, 1, 0)));
  
  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest4(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  CreateShape1(triangles);

  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest5(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1)));
  triangles.push_back(Triangle(Vector3(2, 1, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest6(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1)));
  triangles.push_back(Triangle(Vector3(2, 1, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));
  // Add in a degenerate triangle here. This should not be picked!!
  triangles.push_back(Triangle(Vector3(4, 4, 0), Vector3(4, 4, 0), Vector3(4, 4, 1)));

  TestSplitPick(testName, triangles, file);
}

void SplitPlanePickTest7(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1)));
  triangles.push_back(Triangle(Vector3(2, 1, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));
  // Add in a degenerate triangle here. This should not be picked!!
  triangles.push_back(Triangle(Vector3(4, 4, 0), Vector3(4, 4, 1), Vector3(4, 4, 2)));

  TestSplitPick(testName, triangles, file);
}

void TestBspTreeStructure1(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
  triangles.push_back(Triangle(Vector3(2, 1.5f, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure2(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  CreateShape1(triangles);
  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure3(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  CreateShape2(triangles);
  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure4(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Cube.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure5(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Sphere.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure6(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Octohedron.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, file);
}

void TestBspTreeStructure7(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  TestBspTreeStructure(testName, triangles, file);
}


void PopulateSimpleTree(BspTree& bspTree, float epsilon)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(-1, 2, 0), Vector3(1, 2, 0)));
  triangles.push_back(Triangle(Vector3(0, 0, 0.5f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 3, -0.5f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void PopulateSimpleTree2(BspTree& bspTree, float epsilon)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(0, 1.4f, 0), Vector3(-0.1f, 1.5f, 0), Vector3(0.1f, 1.5f, 0)));
  triangles.push_back(Triangle(Vector3(0, 1.5f, -0.1f), Vector3(-0.1f, 1.6f, 0.1f), Vector3(0.1f, 1.6f, 0.1f)));
  triangles.push_back(Triangle(Vector3(0, 1.3f, -0.1f), Vector3(0.1f, 1.4f, 0.1f), Vector3(-0.1f, 1.4f, 0.1f)));
  triangles.push_back(Triangle(Vector3(0, 0, 0.5f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 3, -0.5f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void PopulateSimpleTree3(BspTree& bspTree, float epsilon, std::vector<Triangle>& triangles)
{
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(-1, 2, 0), Vector3(1, 2, 0)));
  triangles.push_back(Triangle(Vector3(0, 2, 0.2f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 1, -0.2f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  triangles.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 1.8f, 1), Vector3(-1, 1.8f, 1)));
  triangles.push_back(Triangle(Vector3(0, 2.25f, 2), Vector3(1, 4, 2), Vector3(-1, 4, 2)));
  triangles.push_back(Triangle(Vector3(0, 0, -1), Vector3(1, 1.5f, -1), Vector3(-1, 1.5f, -1)));
  triangles.push_back(Triangle(Vector3(0, 0.75f, -1.5f), Vector3(1, 3, -2), Vector3(-1, 3, -2)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void BspSimpleRayCast1(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;
  ray.mDirection = Vector3(0, 0, 1);

  ray.mStart = Vector3(0, 4, -2);
  rays.push_back(ray);

  ray.mStart.y = 2.5f;
  rays.push_back(ray);

  ray.mStart.y = 1.5f;
  rays.push_back(ray);

  ray.mStart.y = 0.5f;
  rays.push_back(ray);

  ray.mStart.y = -1.0f;
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

void BspSimpleRayCast2(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;
  ray.mDirection = Vector3(0, 0, -1);

  ray.mStart = Vector3(0, 4, 2);
  rays.push_back(ray);

  ray.mStart.y = 2.5f;
  rays.push_back(ray);

  ray.mStart.y = 1.5f;
  rays.push_back(ray);

  ray.mStart.y = 0.5f;
  rays.push_back(ray);

  ray.mStart.y = -1.0f;
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

void BspSimpleRayCast3(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;
  ray.mDirection = Math::Normalized(Vector3(0, -1, 1));

  ray.mStart = Vector3(0, 5.75f, -2);
  rays.push_back(ray);

  ray.mStart.y = 5.0f;
  rays.push_back(ray);

  ray.mStart.y = 4.25f;
  rays.push_back(ray);

  ray.mStart.y = 3.5f;
  rays.push_back(ray);

  ray.mStart.y = 2.75f;
  rays.push_back(ray);

  ray.mStart.y = 2.0f;
  rays.push_back(ray);

  ray.mStart.y = 1.25f;
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

void BspSimpleRayCast4(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;
  ray.mDirection = Math::Normalized(Vector3(0, 1, -1));

  ray.mStart = Vector3(0, 1.75f, 2);
  rays.push_back(ray);

  ray.mStart.y = 1.0f;
  rays.push_back(ray);

  ray.mStart.y = 0.25f;
  rays.push_back(ray);

  ray.mStart.y = -0.5;
  rays.push_back(ray);

  ray.mStart.y = -1.25f;
  rays.push_back(ray);

  ray.mStart.y = -2.0f;
  rays.push_back(ray);

  ray.mStart.y = -2.75f;
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

// Test coplanar start
void BspSimpleRayCast5(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.5f, .25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.5f, .25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.5f, -.25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.5f, -.25f);
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

// Test coplanar triangles with coplanar start. 
void BspSimpleRayCast6(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, 5, -1));
  ray.mStart = Vector3(0, -0.5f, .25f);
  rays.push_back(ray);
  
  ray.mDirection = Math::Normalized(Vector3(0, 5, 1));
  ray.mStart = Vector3(0, -0.5f, -.25f);
  rays.push_back(ray);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, -1));
  ray.mStart = Vector3(0, 0.5f, .25f);
  rays.push_back(ray);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, 1));
  ray.mStart = Vector3(0, 0.5f, -.25f);
  rays.push_back(ray);
  
  // Bottom tri
  ray.mDirection = Math::Normalized(Vector3(0, -5, -1));
  ray.mStart = Vector3(0, 3.5f, .25f);
  rays.push_back(ray);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, 1));
  ray.mStart = Vector3(0, 3.5f, -.25f);
  rays.push_back(ray);
  
  ray.mDirection = Math::Normalized(Vector3(0, 5, -1));
  ray.mStart = Vector3(0, 2.5f, .25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, 5, 1));
  ray.mStart = Vector3(0, 2.5f, -.25f);
  rays.push_back(ray);

  // center tri
  ray.mDirection = Math::Normalized(Vector3(0, 3, -1));
  ray.mStart = Vector3(0, 0.75f, .25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, 3, 1));
  ray.mStart = Vector3(0, 0.75f, -.25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, -3, -1));
  ray.mStart = Vector3(0, 2.25f, .25f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, -3, 1));
  ray.mStart = Vector3(0, 2.25f, -.25f);
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

// Test multiple coplanar triangles
void BspSimpleRayCast7(const std::string& testName, FILE* file = nullptr)
{
  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree2(bspTree, epsilon);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));
  
  std::vector<Ray> rays;

  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.7f, 0.05f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.7f, -.05f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.2f, 0.05f);
  rays.push_back(ray);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.2f, -.05f);
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

// Test the last two edge cases (expanded t-values and thick plane check values)
void BspSimpleRayCast8(const std::string& testName, FILE* file = nullptr)
{
  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  PrintTestHeader(file, testName);

  float epsilon = 0.25f;
  BspTree bspTree;
  std::vector<Triangle> triangles;
  PopulateSimpleTree3(bspTree, epsilon, triangles);
  bspTree.DebugDraw(debugDrawLevel, Vector4(1));

  std::vector<Ray> rays;

  Ray ray;
  ray.mStart = Vector3(0, -2.21259999f, -0.609817982f);
  ray.mDirection = Vector3(0, 0.984807730f, 0.173648179f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 1.91507995f, -4.35225010f);
  ray.mDirection = Vector3(0, -0.374606609f, 0.927183867f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 2.53433990f, 1.40890002f);
  ray.mDirection = Vector3(0, -0.374606609f, 0.927183867f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 4.39364004f, -0.431046009f);
  ray.mDirection = Vector3(0, -0.970295727f, 0.241921857f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 3.49269009f, -0.701030016f);
  ray.mDirection = Vector3(0, -0.990268052f, -0.139173105f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 1.32701004f, 2.93155003f);
  ray.mDirection = Vector3(0, 0.224951208f, -0.974370003f);
  rays.push_back(ray);

  ray.mStart = Vector3(0, 0.630509973f, -1.86195004f);
  ray.mDirection = Vector3(0, 0.552694738f, 0.833383799f);
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

// Special case to verify edge case 3.5
void BspSimpleRayCast9(const std::string& testName, FILE* file = nullptr)
{
  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  PrintTestHeader(file, testName);

  float epsilon = 0.5f;
  BspTree bspTree;
  std::vector<Triangle> triangles;
  triangles.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, 0, 3.20108f), Vector3(1, 0, 3.20108f)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(-1, -1, 0), Vector3(1, -1, 0)));
  triangles.push_back(Triangle(Vector3(0, 1.6f, 0.55f), Vector3(-1, -0.4f, 0.55f), Vector3(1, -0.4f, 0.55f)));
  triangles.push_back(Triangle(Vector3(0, 1, -3), Vector3(-1, -1, -3), Vector3(1, -1, -3)));
  bspTree.Construct(triangles, splitWeight, epsilon);

  bspTree.DebugDraw(debugDrawLevel, Vector4(1));


  std::vector<Ray> rays;

  Ray ray;
  ray.mStart = Vector3(0, -.95f, 3.14f);
  ray.mDirection = Vector3(0, 1, -3.965f);
  rays.push_back(ray);

  TestRays(bspTree, rays, epsilon, file);
}

void BspRayCastStressTest1(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList triangles;
  CreateShape1(triangles);

  std::vector<Vector3> rayStarts;
  rayStarts.push_back(Vector3(2, 3.5f, 0.5f));

  TestRaycasts(triangles, rayStarts, 20.0f, file);
}

void BspRayCastStressTest2(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList triangles;
  CreateShape2(triangles);

  std::vector<Vector3> rayStarts;
  rayStarts.push_back(Vector3(1, 1, 0.5f));
  rayStarts.push_back(Vector3(3, 1, 0.5f));

  TestRaycasts(triangles, rayStarts, 10.0f, file);
}

void BspRayCastStressTest3(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Cube.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, file);
}

void BspRayCastStressTest4(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Sphere.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, file);
}

void BspRayCastStressTest5(const std::string& testName, FILE* file = nullptr)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Octohedron.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, file);
}

typedef void (*BspTreeTestFn)(BspTree& tree1, BspTree& tree2, FILE* file);

void TestUnion(BspTree& tree1, BspTree& tree2, FILE* file)
{
  tree1.Union(&tree2, splitWeight, epsilon);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  tree1.DebugDraw(debugDrawLevel, Vector4(0, 0, 1, 1), 3);

  PrintSpatialPartitionStructure(tree1, file, "Union");
}

void TestIntersection(BspTree& tree1, BspTree& tree2, FILE* file)
{
  tree1.Intersection(&tree2, splitWeight, epsilon);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  tree1.DebugDraw(debugDrawLevel, Vector4(0, 0, 1, 1), 3);

  PrintSpatialPartitionStructure(tree1, file, "Intersection");
}

void TestSubtraction(BspTree& tree1, BspTree& tree2, FILE* file)
{
  tree1.Subtract(&tree2, splitWeight, epsilon);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  tree1.DebugDraw(debugDrawLevel, Vector4(0, 0, 1, 1), 3);

  PrintSpatialPartitionStructure(tree1, file, "Subtraction");
}

void TestBspTree(const std::string& testName, TriangleList& tris1, TriangleList& tris2, BspTreeTestFn testFn, FILE* file)
{
  PrintTestHeader(file, testName);

  BspTree tree1;
  tree1.Construct(tris1, splitWeight, epsilon);
  BspTree tree2;
  tree2.Construct(tris2, splitWeight, epsilon);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 1);
  tree2.DebugDraw(debugDrawLevel, Vector4(0, 1, 0, 1), 2);

  testFn(tree1, tree2, file);
}

void TestBspOperations(TriangleList& tris1, TriangleList& tris2, FILE* file)
{
  BspTree tree1;
  tree1.Construct(tris1, splitWeight, epsilon);
  BspTree tree2;
  tree2.Construct(tris2, splitWeight, epsilon);

  int debugDrawLevel = Application::mUnitTestValues.mDebugDrawLevel;
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 1);
  tree2.DebugDraw(debugDrawLevel, Vector4(0, 1, 0, 1), 1);
  PrintSpatialPartitionStructure(tree1, file, "Original Tree1");
  PrintSpatialPartitionStructure(tree2, file, "Original Tree2");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Invert();
  tree2.Invert();
  PrintSpatialPartitionStructure(tree1, file, "Tree1 Inverted");
  PrintSpatialPartitionStructure(tree2, file, "Tree2 Inverted");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.ClipTo(&tree2, epsilon);
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 2);
  PrintSpatialPartitionStructure(tree1, file, "Tree1 Clipped to Tree2");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree2.ClipTo(&tree1, epsilon);
  tree2.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 3);
  PrintSpatialPartitionStructure(tree2, file, "Tree2 Clipped to Tree1");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Union(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 4);
  PrintSpatialPartitionStructure(tree1, file, "Union");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Intersection(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 5);
  PrintSpatialPartitionStructure(tree1, file, "Intersection");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Subtract(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(debugDrawLevel, Vector4(1, 0, 0, 1), 6);
  PrintSpatialPartitionStructure(tree1, file, "Tree1 - Tree2");

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree2.Subtract(&tree1, splitWeight, epsilon);
  tree2.DebugDraw(debugDrawLevel, Vector4(0, 1, 0, 1), 7);
  PrintSpatialPartitionStructure(tree2, file, "Tree2 - Tree1");
}

void BspCsgSimple1(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(-1, 0, 0), Vector3(1, 0, -1), Vector3(-1, 0, 1)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, 0), Vector3(1, 1, 0), Vector3(-1, 1, 0)));
  
  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple2(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, 0, -2), Vector3(1, 0, 2), Vector3(-1, 0, 2)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple3(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, 0.5f), Vector3(1, 1, 0.5f), Vector3(-1, 1, 0.5f)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple4(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, 1, -2), Vector3(1, 2, -1), Vector3(-1, 2, -1)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple5(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, -2), Vector3(1, -2, -1), Vector3(-1, -2, -1)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple6(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -3, -1.5f), Vector3(1, 3, -1.5f), Vector3(-1, 3, -1.5f)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple7(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0,  1), Vector3( 1,  2, -2), Vector3(-1,  2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0,  1), Vector3(-1, -2, -2), Vector3( 1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 2, -2), Vector3( 1, -2, -2), Vector3(-1, -2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3( 1,  2, 2), Vector3(0, 0, -1), Vector3(-1,  2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3( 1, -2, 2)));
  tris2.push_back(Triangle(Vector3( 1, -2, 2), Vector3(1, 2,  2), Vector3(-1, -2, 2)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple8(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 2), Vector3(0, 0, -1), Vector3(-1, 2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 1), Vector3(-1, -2, 2), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 1), Vector3(1, 2, 2), Vector3(-1, 2, 2)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple9(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, 0), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 2), Vector3(0, 0, -1), Vector3(-1, 2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 0), Vector3(-1, -2, 2), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, 2, 2), Vector3(-1, 2, 2)));

  TestBspOperations(tris1, tris2, file);
}

void BspCsgSimple10(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 2.5), Vector3(1, 2, -0.5), Vector3(-1, 2, -0.5)));
  tris1.push_back(Triangle(Vector3(0, 0, 2.5), Vector3(-1, -2, -0.5), Vector3(1, -2, -0.5)));
  tris1.push_back(Triangle(Vector3(1, 0, 0.5), Vector3(1, -2, -0.5), Vector3(-1, -2, -0.5)));
  tris1.push_back(Triangle(Vector3(1, 0, 0.5), Vector3(-1, 2, -0.5), Vector3(1, 2, -0.5)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 0.5), Vector3(0, 0, -2.5), Vector3(-1, 2, 0.5)));
  tris2.push_back(Triangle(Vector3(-1, -2, 0.5), Vector3(0, 0, -2.5), Vector3(1, -2, 0.5)));
  tris2.push_back(Triangle(Vector3(1, 0, -0.5), Vector3(-1, -2, 0.5), Vector3(1, -2, 0.5)));
  tris2.push_back(Triangle(Vector3(1, 0, -0.5), Vector3(1, 2, 0.5), Vector3(-1, 2, 0.5)));

  TestBspOperations(tris1, tris2, file);
}

void BspUnion1(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestUnion, file);
}

void BspIntersection1(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestIntersection, file);
}

void BspSubtraction1(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestSubtraction, file);
}

void BspUnion2(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);
  
  TestBspTree(testName, tris1, tris2, TestUnion, file);
}

void BspIntersection2(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestIntersection, file);
}

void BspSubtraction2(const std::string& testName, FILE* file = nullptr)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestSubtraction, file);
}

void LargeCsg(const std::string& testName, FILE* file = nullptr)
{
  PrintTestHeader(file, testName);

  TriangleList cubeTris;
  Matrix4 cubeTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", cubeTransform, cubeTris);

  TriangleList sphereTris;
  Matrix4 sphereTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1.3f));
  LoadMesh("Sphere.txt", sphereTransform, sphereTris);

  TriangleList cylXTris;
  Matrix4 cylXTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 1), 0.5f * Math::cPi), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylXTransform, cylXTris);

  TriangleList cylYTris;
  Matrix4 cylYTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 1), 0.0f), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylYTransform, cylYTris);

  TriangleList cylZTris;
  Matrix4 cylZTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(1, 0, 0), 0.5f * Math::cPi), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylZTransform, cylZTris);

  float epsilon = 0.0001f;

  BspTree cubeTree;
  cubeTree.Construct(cubeTris, splitWeight, epsilon);
  //cubeTree.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Vector4(1), 1);

  BspTree sphereTree;
  sphereTree.Construct(sphereTris, splitWeight, epsilon);

  BspTree cylXTree;
  cylXTree.Construct(cylXTris, splitWeight, epsilon);
  BspTree cylYTree;
  cylYTree.Construct(cylYTris, splitWeight, epsilon);
  BspTree cylZTree;
  cylZTree.Construct(cylZTris, splitWeight, epsilon);
  
  
  cubeTree.Intersection(&sphereTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Vector4(1), 2);

  cubeTree.Subtract(&cylXTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Vector4(1), 3);

  cubeTree.Subtract(&cylYTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Vector4(1), 4);

  cubeTree.Subtract(&cylZTree, splitWeight, epsilon);
  cubeTree.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Vector4(1), 5);

  PrintSpatialPartitionStructure(cubeTree, file, "Triangles");
}

void RegisterTriangleSplitTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TriangleSplitTest1, list);
  DeclareSimpleUnitTest(TriangleSplitTest2, list);
  DeclareSimpleUnitTest(TriangleSplitTest3, list);
  DeclareSimpleUnitTest(TriangleSplitTest4, list);
  DeclareSimpleUnitTest(TriangleSplitTest5, list);
  DeclareSimpleUnitTest(TriangleSplitTest6, list);
  DeclareSimpleUnitTest(TriangleSplitTest7, list);
  DeclareSimpleUnitTest(TriangleSplitTest8, list);
  DeclareSimpleUnitTest(TriangleSplitTest9, list);
  DeclareSimpleUnitTest(TriangleSplitTest10, list);
  DeclareSimpleUnitTest(TriangleSplitTest11, list);
  DeclareSimpleUnitTest(TriangleSplitTest12, list);
  DeclareSimpleUnitTest(TriangleSplitTest13, list);
  DeclareSimpleUnitTest(TriangleSplitTest14, list);
  DeclareSimpleUnitTest(TriangleSplitTest15, list);
  DeclareSimpleUnitTest(TriangleSplitTest16, list);
  DeclareSimpleUnitTest(TriangleSplitTest17, list);
  DeclareSimpleUnitTest(TriangleSplitTest18, list);
  DeclareSimpleUnitTest(TriangleSplitTest19, list);
  DeclareSimpleUnitTest(TriangleSplitTest20, list);
  DeclareSimpleUnitTest(TriangleSplitTest21, list);
  DeclareSimpleUnitTest(TriangleSplitTest22, list);
  DeclareSimpleUnitTest(TriangleSplitTest23, list);
  DeclareSimpleUnitTest(TriangleSplitTest24, list);
  DeclareSimpleUnitTest(TriangleSplitTest25, list);
  DeclareSimpleUnitTest(TriangleSplitTest26, list);
  DeclareSimpleUnitTest(TriangleSplitTest27, list);
  DeclareSimpleUnitTest(TriangleSplitTest28, list);
  DeclareSimpleUnitTest(TriangleSplitTest29, list);
  DeclareSimpleUnitTest(TriangleSplitTest30, list);
  DeclareSimpleUnitTest(TriangleSplitTest31, list);
  DeclareSimpleUnitTest(TriangleSplitTest32, list);
  DeclareSimpleUnitTest(TriangleSplitTest33, list);
}

void RegisterSplitPlanePickTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SplitPlanePickTest1, list);
  DeclareSimpleUnitTest(SplitPlanePickTest2, list);
  DeclareSimpleUnitTest(SplitPlanePickTest3, list);
  DeclareSimpleUnitTest(SplitPlanePickTest4, list);
  DeclareSimpleUnitTest(SplitPlanePickTest5, list);
  DeclareSimpleUnitTest(SplitPlanePickTest6, list);
  DeclareSimpleUnitTest(SplitPlanePickTest7, list);
}

void RegisterTreeConstructionTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TestBspTreeStructure1, list);
  DeclareSimpleUnitTest(TestBspTreeStructure2, list);
  DeclareSimpleUnitTest(TestBspTreeStructure3, list);
  DeclareSimpleUnitTest(TestBspTreeStructure4, list);
  DeclareSimpleUnitTest(TestBspTreeStructure5, list);
  DeclareSimpleUnitTest(TestBspTreeStructure6, list);
  DeclareSimpleUnitTest(TestBspTreeStructure7, list);
}

void RegisterRayCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BspSimpleRayCast1, list);
  DeclareSimpleUnitTest(BspSimpleRayCast2, list);
  DeclareSimpleUnitTest(BspSimpleRayCast3, list);
  DeclareSimpleUnitTest(BspSimpleRayCast4, list);
  DeclareSimpleUnitTest(BspSimpleRayCast5, list);
  DeclareSimpleUnitTest(BspSimpleRayCast6, list);
  DeclareSimpleUnitTest(BspSimpleRayCast7, list);
  DeclareSimpleUnitTest(BspSimpleRayCast8, list);
  DeclareSimpleUnitTest(BspSimpleRayCast9, list);
  DeclareSimpleUnitTest(BspRayCastStressTest1, list);
  DeclareSimpleUnitTest(BspRayCastStressTest2, list);
  DeclareSimpleUnitTest(BspRayCastStressTest3, list);
  DeclareSimpleUnitTest(BspRayCastStressTest4, list);
  DeclareSimpleUnitTest(BspRayCastStressTest5, list);
}

void RegisterCsgTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BspCsgSimple1, list);
  DeclareSimpleUnitTest(BspCsgSimple2, list);
  DeclareSimpleUnitTest(BspCsgSimple3, list);
  DeclareSimpleUnitTest(BspCsgSimple4, list);
  DeclareSimpleUnitTest(BspCsgSimple5, list);
  DeclareSimpleUnitTest(BspCsgSimple6, list);
  DeclareSimpleUnitTest(BspCsgSimple7, list);
  DeclareSimpleUnitTest(BspCsgSimple8, list);
  DeclareSimpleUnitTest(BspCsgSimple9, list);
  DeclareSimpleUnitTest(BspCsgSimple10, list);
  DeclareSimpleUnitTest(BspUnion1, list);
  DeclareSimpleUnitTest(BspIntersection1, list);
  DeclareSimpleUnitTest(BspSubtraction1, list);
  DeclareSimpleUnitTest(BspUnion2, list);
  DeclareSimpleUnitTest(BspIntersection2, list);
  DeclareSimpleUnitTest(BspSubtraction2, list);
  //DeclareSimpleUnitTest(LargeCsg, list);
}

void InitializeAssignment4Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns.back();

  RegisterTriangleSplitTests(list);
  RegisterSplitPlanePickTests(list);
  RegisterTreeConstructionTests(list);
  RegisterRayCastTests(list);
  RegisterCsgTests(list); 
}
