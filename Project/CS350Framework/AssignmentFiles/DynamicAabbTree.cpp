///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//--------------------------------------------------------------------DynamicAabbTreeNode
DynamicAabbTreeNode* DynamicAabbTreeNode::GetParent() const
{
  return mParent;
}

DynamicAabbTreeNode* DynamicAabbTreeNode::GetLeftChild() const
{
  return mLeft;
}

DynamicAabbTreeNode* DynamicAabbTreeNode::GetRightChild() const
{
  return mRight;
}

Aabb DynamicAabbTreeNode::GetAabb() const
{
  return mAabb;
}

void* DynamicAabbTreeNode::GetClientData() const
{
  return mClientData;
}

int DynamicAabbTreeNode::GetHeight() const
{
  return mHeight;
}

bool DynamicAabbTreeNode::IsLeaf() const
{
  return mLeft == nullptr && mRight == nullptr;
}


bool DynamicAabbTreeNode::IsLeftChild() const
{
  if (mParent)
  {
    return this == mParent->mLeft;
  }
  return false;
}

bool DynamicAabbTreeNode::IsRightChild() const
{
  return !IsLeftChild();
}

//--------------------------------------------------------------------DynamicAabbTree
const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
{
  mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
  // depth first delete because it doesn't matter how we 'recurse'
  // and stacks are generally faster than queues
  std::stack<Node*> deletionStack;
  deletionStack.push(mRoot);
  while (!deletionStack.empty())
  {
    Node * toDel = deletionStack.top();
    deletionStack.pop();
    if (toDel->mLeft) deletionStack.push(toDel->mLeft);
    if (toDel->mRight) deletionStack.push(toDel->mRight);
    delete toDel;
  }
  
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  Node* newNode = new Node;
  newNode->mAabb = Aabb::BuildFromCenterAndHalfExtents(data.mAabb.GetCenter()
                  , data.mAabb.GetHalfSize() + Vector3(mFatteningFactor));
  newNode->mClientData = data.mClientData;

  key.mVoidKey = newNode;

  // no neeed to balane if this is the first node
  if (mRoot == nullptr)
  {
    mRoot = newNode;
    return;
  }


  Node* walker = mRoot;
  while (!walker->IsLeaf())
  {
    Node* left = walker->mLeft;
    Node* right = walker->mRight;

    float saL = Aabb::Combine(left->mAabb, newNode->mAabb).GetSurfaceArea();
    float saR = Aabb::Combine(right->mAabb, newNode->mAabb).GetSurfaceArea();

    saL > saR ? walker = left : walker = right;
  }

  Node* splitNode = new Node;
  splitNode->mLeft = walker;
  splitNode->mRight = newNode;

  if (walker->mParent)
  {
    walker->IsLeftChild() ? walker->mParent->mLeft = splitNode : walker->mParent->mRight = splitNode;
  }

  walker->mParent = splitNode;
  newNode->mParent = splitNode;
  
  Reshape(splitNode);
  Balance(splitNode);
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  Node* node = (Node*)key.mVoidKey;
  if (node->mAabb.Contains(data.mAabb))
  {
    node->mClientData = data.mClientData;
    return;
  }

  RemoveData(key);
  InsertData(key, data);
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
}

DynamicAabbTreeNode* DynamicAabbTree::GetRoot() const
{
  /******Student:Assignment3******/
  // Return the root of your tree so that unit tests can print out the contents
  /******Student:Assignment3******/
  Warn("Assignment3: Required function un-implemented");
  return nullptr;
}

void DynamicAabbTree::Reshape(Node * node)
{
}

void DynamicAabbTree::Balance(Node * node)
{
}

