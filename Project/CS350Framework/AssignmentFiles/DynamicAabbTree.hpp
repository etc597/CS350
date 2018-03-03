///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

//--------------------------------------------------------------------DynamicAabbTreeNode
class DynamicAabbTreeNode
{
public:
  DynamicAabbTreeNode* GetParent() const;
  DynamicAabbTreeNode* GetLeftChild() const;
  DynamicAabbTreeNode* GetRightChild() const;
  Aabb GetAabb() const;
  void* GetClientData() const;
  int GetHeight() const;

  // Add you implementation here
  bool IsLeaf() const;
  bool IsLeftChild() const;
  bool IsRightChild() const;
  DynamicAabbTreeNode* GetSibling() const;

  DynamicAabbTreeNode* mParent = nullptr;
  DynamicAabbTreeNode* mLeft   = nullptr;
  DynamicAabbTreeNode* mRight  = nullptr;
  Aabb mAabb                   = Aabb();
  void * mClientData           = nullptr;
  int mHeight                  = 0;
  size_t mLastAxis             = 0;
};

//--------------------------------------------------------------------DynamicAabbTree
/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:

  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  DynamicAabbTreeNode* GetRoot() const;

  // A fattening factor to use for insertion to prevent jitter from causing updates
  static const float mFatteningFactor;

  // Add your implementation here
private:
  typedef DynamicAabbTreeNode Node;

  void Reshape(Node* node);
  void Balance(Node* node);


  Node* mRoot;

};
