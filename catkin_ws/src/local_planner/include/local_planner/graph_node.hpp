#pragma once

typedef std::pair<double, double> point;

namespace Prius {

struct GraphNode{

    point child_point;
    point parent_point;
    double heading;
    double velocity;
    double g = 0;
    double cost;

    GraphNode(const point &cp, const point &pp, const double &h, const double &v)
    {
        child_point = cp;
        parent_point = pp;
        heading = h;
        velocity = v;
    }

    bool operator==(const GraphNode &rhs) const
    {
        return child_point == rhs.child_point &&
               parent_point == rhs.parent_point &&
               heading == rhs.heading &&
               velocity == rhs.velocity;
    }
};

}
