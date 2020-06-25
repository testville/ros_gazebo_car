#include <tf/tf.h>
namespace robot_state {
    class State {
    public:
        tf::Vector3 pose;
        float theta = 0.0;
        State *parent = NULL;
        float cost = 0.0;
        State(tf::Vector3 pose, float theta)
        {
            this->pose = pose;
            this->theta = theta;
        }
        State(tf::Vector3 pose, float theta, State *parent)
        {
            this->pose = pose;
            this->theta = theta;
            this->parent = parent;
        }
        State(tf::Vector3 pose, float theta, State *parent, float cost)
        {
            this->pose = pose;
            this->theta = theta;
            this->parent = parent;
            this->cost = cost;
        }
        State()
        {
            this->pose = tf::Vector3();
            this->theta = 0.0;
        }

        State(const State& obj)
        {
            this->pose = obj.pose;
            this->theta = obj.theta;
            this->parent = obj.parent;
            this->cost = obj.cost;
        }

        void add(State const &obj1)
        {
            this->pose += obj1.pose;
            this->theta += obj1.theta;
        }


    };
    State operator+(State const &obj1, State const &obj2)
    {
        return State(obj1.pose + obj2.pose, obj1.theta + obj2.theta);
    }

    State* operator+(State *obj1, State obj2)
    {
        return new State(obj1->pose + obj2.pose, obj1->theta + obj2.theta);
    }

    State operator* (float x, const State& obj)
    {
        return State(obj.pose * x, obj.theta * x);
    }

    State operator* (const State& obj, float x)
    {
        return State(obj.pose * x, obj.theta * x);
    }
}