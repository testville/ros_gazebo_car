#ifndef KD_TREE
#define KD_TREE

#include <algorithm>
#include <vector>
namespace kd_tree
{

template <class T>
class KDNode
{
public:
    T data;
    std::vector<float> pos;
    int axis;
    KDNode *left, *right;
    KDNode(T data, std::pair<float, float> pos, int axis);
    KDNode(T data, std::vector<float> pos, int axis);

private:
};

template <class T>
KDNode<T>::KDNode(T data, std::pair<float, float> pos, int axis)
{
    this->data = data;
    this->pos = std::vector<float>();
    this->pos.push_back(pos.first);
    this->pos.push_back(pos.second);
    this->axis = axis;
    this->left = this->right = NULL;
}

template <class T>
KDNode<T>::KDNode(T data, std::vector<float> pos, int axis)
{
    this->data = data;
    this->pos = pos;
    this->axis = axis;
    this->left = this->right = NULL;
}

template <class T>
class KDTree
{
private:
    KDNode<T> *head;
    kd_tree::KDNode<T> *insertRec(kd_tree::KDNode<T> *Node, std::vector<float> pos, T data, unsigned depth);
    void NNSearchRec(kd_tree::KDNode<T> *node, std::vector<float> queryPoint, kd_tree::KDNode<T> *&currentBest, float &bestDistance);
    void RangeSearchRec(std::vector<std::vector<float>> queryRange, kd_tree::KDNode<T> *node, std::vector<kd_tree::KDNode<T> *> &v);
public:
    KDTree();
    void insert(std::pair<float, float> pos, T data);
    void NNSearch(std::pair<float, float> queryPoint, kd_tree::KDNode<T> *&n);
    void RangeSearch(std::pair<float, float> queryRangeX, std::pair<float, float> queryRangeY, std::vector<kd_tree::KDNode<T> *> &v);
};

template <class T>
KDTree<T>::KDTree()
{
    this->head = NULL;
}

template <class T>
void KDTree<T>::insert(std::pair<float, float> pos, T data)
{

    std::vector<float> v;
    v.push_back(pos.first);
    v.push_back(pos.second);
    if (this->head == NULL)
    {
        this->head = (new kd_tree::KDNode<T>(data, pos, 0));
    }
    else
    {
        this->insertRec(this->head, v, data, 0);
    }
}

template <class T>
kd_tree::KDNode<T> *KDTree<T>::insertRec(kd_tree::KDNode<T> *Node, std::vector<float> pos, T data, unsigned depth)
{
    if (Node == NULL)
        return (new kd_tree::KDNode<T>(data, pos, depth % 2));
    // Calculate current dimension (cd) of comparison
    unsigned cd = depth % 2;

    if (pos[cd] < Node->pos[cd])
    {
        Node->left = this->insertRec(Node->left, pos, data, depth + 1);
    }
    else
    {
        Node->right = this->insertRec(Node->right, pos, data, depth + 1);
    }

    return Node;
}

template <class T>
void KDTree<T>::NNSearch(std::pair<float, float> queryPoint, kd_tree::KDNode<T> *&n)
{
    std::vector<float> v;
    v.push_back(queryPoint.first);
    v.push_back(queryPoint.second);
    float disBig = 100000000000.0;
    this->NNSearchRec(this->head, v, n, disBig);
}

static float dist(std::vector<float> s, std::vector<float> e)
{
    float a = e[1] - s[1];
    float b = e[0] - s[0];
    return sqrt(a * a + b * b);
}

template <class T>
void KDTree<T>::NNSearchRec(kd_tree::KDNode<T> *node, std::vector<float> queryPoint, kd_tree::KDNode<T> *&currentBest, float &bestDistance)
{
    float d = kd_tree::dist(node->pos, queryPoint);
    if (d < bestDistance)
    {
        currentBest = node;
        bestDistance = d;
    }

    if (node->left != NULL || node->right != NULL)
    {
        if (queryPoint[node->axis] <= node->pos[node->axis])
        {
            if (queryPoint[node->axis] - bestDistance <= node->pos[node->axis] && node->left != NULL)
            {
                this->NNSearchRec(node->left, queryPoint, currentBest, bestDistance);
            }
            if (queryPoint[node->axis] + bestDistance > node->pos[node->axis] && node->right != NULL)
            {
                this->NNSearchRec(node->right, queryPoint, currentBest, bestDistance);
            }
        }
        else
        {
            if (queryPoint[node->axis] + bestDistance > node->pos[node->axis] && node->right != NULL)
            {
                this->NNSearchRec(node->right, queryPoint, currentBest, bestDistance);
            }
            if (queryPoint[node->axis] - bestDistance <= node->pos[node->axis] && node->left != NULL)
            {
                this->NNSearchRec(node->left, queryPoint, currentBest, bestDistance);
            }
        }
    }


}

template <class T>
void KDTree<T>::RangeSearch(std::pair<float, float> queryRangeX, std::pair<float, float> queryRangeY, std::vector<kd_tree::KDNode<T> *> &v)
{
    std::vector<float> xRange;
    std::vector<float> yRange;
    xRange.push_back(queryRangeX.first);
    xRange.push_back(queryRangeX.second);

    yRange.push_back(queryRangeY.first);
    yRange.push_back(queryRangeY.second);
    std::vector<std::vector<float>> queryRange;
    queryRange.push_back(xRange);
    queryRange.push_back(yRange);
    this->RangeSearchRec(queryRange, this->head, v);
}

template <class T>
void KDTree<T>::RangeSearchRec(std::vector <std::vector<float>> queryRange, kd_tree::KDNode<T> *node, std::vector<kd_tree::KDNode<T> *> &v)
{
    if((queryRange[0][0] <= node->pos[0] && queryRange[0][1] >= node->pos[0]) && (queryRange[1][0] <= node->pos[1] && queryRange[1][1] >= node->pos[1]))
    {
        v.push_back(node);
    }

    if(node->pos[node->axis] <= queryRange[node->axis][1] && node->right != NULL) //left
    {
        this->RangeSearchRec(queryRange, node->right, v);
    }
    if(node->pos[node->axis] >= queryRange[node->axis][0] && node->left != NULL) //right
    {
        this->RangeSearchRec(queryRange, node->left, v);
    }
    
}

} // namespace kd_tree
#endif // KD_TREE