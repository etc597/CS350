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

DynamicAabbTreeNode * DynamicAabbTreeNode::GetSibling() const
{
  if (mParent)
  {
    return IsLeftChild() ? mParent->mRight : mParent->mLeft;
  }
  return nullptr;
}

//--------------------------------------------------------------------DynamicAabbTree
const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
  : mRoot(nullptr)
{
  mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
  // depth first delete because it doesn't matter how we 'recurse'
  // and stacks are generally faster than queues
  std::stack<Node*> deletionStack;
  if(mRoot) deletionStack.push(mRoot);
  while (!deletionStack.empty())
  {
    Node * toDel = deletionStack.top();
    deletionStack.pop();
    if (!toDel->IsLeaf())
    {
      deletionStack.push(toDel->mLeft);
      deletionStack.push(toDel->mRight);
    }
    delete toDel;
  }
  
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  Node* newNode = new Node;
  newNode->mAabb = Aabb::BuildFromCenterAndHalfExtents(data.mAabb.GetCenter()
                  , data.mAabb.GetHalfSize() * mFatteningFactor);
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

    // smallest delta surface area
    float saL = Aabb::Combine(left->mAabb, newNode->mAabb).GetSurfaceArea() - left->mAabb.GetSurfaceArea();
    float saR = Aabb::Combine(right->mAabb, newNode->mAabb).GetSurfaceArea() - right->mAabb.GetSurfaceArea();

    saL < saR ? walker = left : walker = right;
  }

  Node* splitNode = new Node;
  splitNode->mLeft = walker;
  splitNode->mRight = newNode;

  splitNode->mParent = walker->mParent;
  if (walker->mParent)
  {
    walker->IsLeftChild() ? walker->mParent->mLeft = splitNode : walker->mParent->mRight = splitNode;
  }
  else
  {
    mRoot = splitNode;
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
  Node* node = (Node*)key.mVoidKey;
  Node* parent = node->mParent;
  // if our parent is null, we are the root. delete
  if (parent == nullptr)
  {
    delete mRoot;
    mRoot = nullptr;
    return;
  }

  Node* sibling = node->GetSibling();
  Node* grandparent = parent->mParent;

  // if our grandparent is null, our sibling will become root
  if (grandparent == nullptr)
  {
    mRoot = sibling;
    sibling->mParent = nullptr;
    delete node;
    delete parent;
    return;
  }

  parent->IsLeftChild() ? grandparent->mLeft = sibling : grandparent->mRight = sibling;
  sibling->mParent = grandparent;

  delete node;
  delete parent;

  Reshape(grandparent);
  Balance(grandparent);
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  // breadth first until we have all nodes of the level
  std::stack<std::pair<Node*, int>> nodes;
  if (mRoot) nodes.emplace(mRoot, 0);
  while (!nodes.empty())
  {
    auto pair = nodes.top();
    nodes.pop();
    Node* top = pair.first;
    int lvl = pair.second;
    if (!top->IsLeaf() && (level == -1 || lvl < level))
    {
      nodes.emplace(top->mLeft, lvl + 1);
      nodes.emplace(top->mRight, lvl + 1);
    }

    if (level == -1 || level == lvl)
    {
      top->mAabb.DebugDraw().SetTransform(transform).Color(color).SetMaskBit(bitMask);
    }
  }
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  std::stack<Node*> rayStack;
  if (mRoot) rayStack.push(mRoot);
  while (!rayStack.empty())
  {
    Node* top = rayStack.top();
    rayStack.pop();
    float t = 0.0f;
    if (RayAabb(ray.mStart, ray.mDirection, top->mAabb.GetMin(), top->mAabb.GetMax(), t))
    {
      if (top->IsLeaf())
      {
        results.AddResult(CastResult(top->mClientData, t));
      }
      else
      {
        rayStack.push(top->mLeft);
        rayStack.push(top->mRight);
      }
    }
  }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  std::stack<Node*> frustumStack;
  if (mRoot) frustumStack.push(mRoot);
  while (!frustumStack.empty())
  {
    Node* top = frustumStack.top();
    frustumStack.pop();
    size_t lastAxis = top->mLastAxis;
    auto intersection = FrustumAabb(frustum.GetPlanes(), top->mAabb.GetMin(), top->mAabb.GetMax(), lastAxis);
    if (intersection == IntersectionType::Inside)
    {
      // Node is inside, add all children
      std::stack<Node*> children;
      children.push(top);
      while (!children.empty())
      {
        Node* child = children.top();
        children.pop();
        if (!child->IsLeaf())
        {
          children.push(child->mLeft);
          children.push(child->mRight);
        }
        else
        {
          results.AddResult(CastResult(child->mClientData));
        }
      }
    }
    else if (intersection != IntersectionType::Outside)
    {
      if (top->IsLeaf())
      {
        results.AddResult(CastResult(top->mClientData));
      }
      else
      {
        frustumStack.push(top->mLeft);
        frustumStack.push(top->mRight);
      }
    }
    else
    {
      top->mLastAxis = lastAxis;
    }
  }
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
  std::stack<Node*> queryNodes;
  std::stack<std::pair<Node*, Node*>> queryPairs;

  if (mRoot) queryNodes.push(mRoot);

  while (!queryNodes.empty())
  {
    Node* top = queryNodes.top();
    queryNodes.pop();

    if (top->IsLeaf()) continue;

    queryPairs.emplace(top->mLeft, top->mRight);
    queryNodes.push(top->mLeft);
    queryNodes.push(top->mRight);

    while (!queryPairs.empty())
    {
      auto pair = queryPairs.top();
      queryPairs.pop();

      Node* nodeA = pair.first;
      Node* nodeB = pair.second;

      if (!AabbAabb(nodeA->mAabb.GetMin(), nodeA->mAabb.GetMax(), nodeB->mAabb.GetMin()
        , nodeB->mAabb.GetMax()))
      {
        continue;
      }

      // case 1: both leaf                  --> compare aabbs, add to results
      if (nodeA->IsLeaf() && nodeB->IsLeaf())
      {
        results.AddResult(QueryResult(nodeA->mClientData, nodeB->mClientData));
      }
      // case 2: one leaf, one internal     --> test internal nodes children against the leaf (pairs)
      else if (nodeA->IsLeaf() || nodeB->IsLeaf())
      {
        Node* leaf = nodeA->IsLeaf() ? nodeA : nodeB;
        Node* ntrn = nodeA->IsLeaf() ? nodeB : nodeA;

        queryPairs.emplace(leaf, ntrn->mLeft);
        queryPairs.emplace(leaf, ntrn->mRight);
      }
      // case 3: both internal, split nodes --> split nodes
      else
      {
        SplitNodes(nodeA, nodeB, queryPairs);
      }
    }
  }
}

DynamicAabbTreeNode* DynamicAabbTree::GetRoot() const
{
  return mRoot;
}

void DynamicAabbTree::Reshape(Node * node)
{
  while (node)
  {
    Node* left = node->mLeft;
    Node* right = node->mRight;
    node->mHeight = std::max(left->mHeight, right->mHeight) + 1;
    node->mAabb = Aabb::Combine(left->mAabb, right->mAabb);
    node = node->mParent;
  }
}

void DynamicAabbTree::Balance(Node * node)
{
  while (node)
  {
    Node* left = node->mLeft;
    Node* right = node->mRight;

    if (std::abs(left->mHeight - right->mHeight) > 1)
    {
      // actually balance tree
      Node* pivot = left->mHeight > right->mHeight ? left : right;
      left = pivot->mLeft;
      right = pivot->mRight;

      Node* largeChild = left->mHeight >= right->mHeight ? left : right;
      Node* smallChild = largeChild->GetSibling();

      Node* grandparent = node->mParent;
      pivot->mParent = grandparent;
      if (grandparent)
      {
        node->IsLeftChild() ? grandparent->mLeft = pivot : grandparent->mRight = pivot;
      }
      else
      {
        mRoot = pivot;
      }

      // replace small child with old parent
      smallChild->IsLeftChild() ? pivot->mLeft = node : pivot->mRight = node;
      node->mParent = pivot;

      // replace pivot with small child
      node->mLeft == pivot ? node->mLeft = smallChild : node->mRight = smallChild;
      smallChild->mParent = node;

      Reshape(node); // fix aabbs and heights

      //node = pivot; // pivot is now balanced, we should continue going up from this point
    }

    node = node->mParent;
  }
}

void DynamicAabbTree::SplitNodes(Node * nodeA, Node * nodeB, std::stack<std::pair<Node*, Node*>>& stack)
{
  if (nodeA->mAabb.GetVolume() > nodeB->mAabb.GetVolume())
  {
    stack.emplace(nodeB, nodeA->mLeft);
    stack.emplace(nodeB, nodeA->mRight);
  }
  else
  {
    stack.emplace(nodeA, nodeB->mLeft);
    stack.emplace(nodeA, nodeB->mRight);
  }
}
