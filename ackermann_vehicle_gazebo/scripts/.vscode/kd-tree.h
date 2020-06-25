 #include <algorithm>
 #include "global_planner.h"

template<class T>
class KDNode
{
privae:
    T* data;
    std::pair<float, float> pos;
    KDNode *left, *right;
public
    KDNode(T* data, std::pair<float, float> pos);
};

template<class T>
KDNode<T>::KDNode(T* data, std::pair<float, float> pos){
    this->data = data;
    this->pos = pos;
    this->left = this->right = NULL;
}


class KDTree
{
private:
    KDNode<> head;
public:
    KDTree(/* args */);
};

KDTree::KDTree(/* args */)
{
    this->head = NULL;
};
