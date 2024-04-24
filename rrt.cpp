#include "rrt.h"
//header

namespace rrt_planner
{
 
RRTPlanner::RRTPlanner():
    frame_("map"), iteration_(1000), sample_num_(5), extend_step_(0.2), threhold_(0.1), probability_(0.5)
{
    ros::NodeHandle nd;
    map_sub_ = nd.subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&RRTPlanner::mapCB, this, _1));
    goal_sub_ = nd.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&RRTPlanner::goalCB, this, _1));
    pose_sub_ = nd.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&RRTPlanner::initialPoseCB, this, _1));
    plan_pub_ = nd.advertise<nav_msgs::Path>("rrt_path", 1);
    tree_pub_ = nd.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
    root_ = new TreeNode;
    root_->father = NULL;
    root_->children.clear();
}
 
RRTPlanner::~RRTPlanner()
{
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        delete rrt_nodes_[i];
    }
}
 
bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{    
    if(buildRRT())
    {
        ROS_INFO("Plan found.");
        std::vector<geometry_msgs::PoseStamped> temp_plan;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = goal_node_->cell.x * map_resolution_ + map_origin_x_;
        pose.pose.position.y = goal_node_->cell.y * map_resolution_ + map_origin_y_;
        pose.pose.orientation.w = 1;
        temp_plan.push_back(pose);
        TreeNode* father = goal_node_->father;
        while(father != NULL)
        {
            pose.header.frame_id = "map";
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father;
        }
        plan.clear();
        for(int i = temp_plan.size() - 1; i > -1; i--)
        {
            plan.push_back(temp_plan[i]);
        }
        
        publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
        
        sensor_msgs::PointCloud tree;
        tree.header.frame_id = "map";
        for(int i = 0; i < rrt_nodes_.size(); i++)
        {
            geometry_msgs::Point32 point;
            point.x = rrt_nodes_[i]->cell.x * map_resolution_ + map_origin_x_;
            point.y = rrt_nodes_[i]->cell.y * map_resolution_ + map_origin_y_;
            tree.points.push_back(point);
        }
        tree_pub_.publish(tree);
        return true;
    }
    else
    {
        ROS_ERROR("We can not find a plan in %d cycles.", iteration_);
        
        sensor_msgs::PointCloud tree;
        tree.header.frame_id = "map";
        for(int i = 0; i < rrt_nodes_.size(); i++)
        {
            geometry_msgs::Point32 point;
            point.x = rrt_nodes_[i]->cell.x * map_resolution_ + map_origin_x_;
            point.y = rrt_nodes_[i]->cell.y * map_resolution_ + map_origin_y_;
            tree.points.push_back(point);
        }
        tree_pub_.publish(tree);
        return false;
    }
    
}
 
bool RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a)
{
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    
    if(!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }
    
    plan_pub_.publish(gui_path);
}
 
bool RRTPlanner::buildRRT()
{
    for(int i = 0; i < iteration_; i++)
    {
        if(drawSample())
        {
            for(int j = 0; j < current_sampling_.size(); j++)
            {
                if(extendNearestNode(current_sampling_[j]))
                    return true;
            }
        }
    }
    return false;
}
 
bool RRTPlanner::drawSample()
{
    if(freespace_.size() == 0)
        return false;
    else
    {
        probSample();
    }
    return true;
}
 
bool RRTPlanner::extendNearestNode(PointCell random)
{
    int j;
    double path_length = 10000000;
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        if(hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_[i]->cell.x - random.x, rrt_nodes_[i]->cell.y - random.y);
            j = i;
        }
    }
    TreeNode* node;
    node = new TreeNode;
    if(extendSingleStep(rrt_nodes_[j], node, random))
    {
        node->father = rrt_nodes_[j];
        rrt_nodes_[j]->children.push_back(node);
        rrt_nodes_.push_back(node);
        if(goalReached(node))
        {
            goal_node_ = node;
            return true;
        }
    }
    return false;
}
 
bool RRTPlanner::extendSingleStep(const TreeNode* rrtnode, TreeNode* &node, const PointCell random)
{
    double sintheta, costheta;
    
    sintheta = (random.y - rrtnode->cell.y) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    costheta = (random.x - rrtnode->cell.x) / sqrt(pow((random.x - rrtnode->cell.x), 2) + pow((random.y - rrtnode->cell.y), 2));
    node->cell.x = rrtnode->cell.x + (extend_step_ / map_resolution_) * costheta;
    node->cell.y = rrtnode->cell.y + (extend_step_ / map_resolution_) * sintheta;
    if(map_info_[node->cell.y * map_sizex_ + node->cell.x] == 0)
        return true;
    else
        return false;
}
 
void RRTPlanner::probSample()
{    
    double prob = rd_() % 10 * 0.1;
    
    if(prob > probability_)
    {
        addGoalOrientNode();
    }
    else
        addRandomNode();
}
 
void RRTPlanner::addGoalOrientNode()
{
    current_sampling_.clear();
    for(int i = 0;i < sample_num_;i++)
    {
        PointCell p;
        //这里我们可以将撒的点直接设在终点，也可以在终点附近有一个高斯分布
        p.x = goal_x_;// + (rd_() % 200 - 100) * 0.2;
        p.y = goal_y_;// + (rd_() % 200 - 100) * 0.2;
        current_sampling_.push_back(p);
    }
}
 
void RRTPlanner::addRandomNode()
{
    current_sampling_.clear();
    for(int i = 0;i < sample_num_;i++)
    {
        int x = rd_() % map_sizex_;
        int y = rd_() % map_sizey_;
        
        PointCell p;
        
        p.x = x;
        p.y = y;
        
        current_sampling_.push_back(p);
    }
}
 
bool RRTPlanner::goalReached(const TreeNode* nearestNode)
{
    if(sqrt(pow(nearestNode->cell.x - goal_x_, 2) + pow(nearestNode->cell.y - goal_y_, 2)) * map_resolution_ < threhold_)
        return true;
    else
        return false;
}
 
void RRTPlanner::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    mutex.lock();
        
    map_resolution_ = msg->info.resolution;
    map_sizex_ = msg->info.width;
    map_sizey_ = msg->info.height;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    freespace_.clear();
    int j = 0;
    int index;
    for(int i = 0; i < map_sizex_ * map_sizey_; i++)
    {
        map_info_.push_back(msg->data[i]);
        
        if(msg->data[i] == 0)
        {
            index = i + 1;
            ++j;
            
            PointCell cell;
            cell.x = index % map_sizex_;
            cell.y = index / map_sizex_;
            freespace_[j] = cell;
        }
        
    }
    mutex.unlock();
}
 
void RRTPlanner::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    mutex.lock();
    if(!map_info_.empty())
    {
        start_x_ = (msg->pose.pose.position.x - map_origin_x_) / map_resolution_;
        start_y_ = (msg->pose.pose.position.y - map_origin_y_) / map_resolution_;
        start_.pose = msg->pose.pose;
        
        root_->cell.x = start_x_;
        root_->cell.y = start_y_;
        rrt_nodes_.push_back(root_);
    }
    mutex.unlock();
}
 
void RRTPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mutex.lock();
    if(!map_info_.empty())
    {        
        goal_x_ = (msg->pose.position.x - map_origin_x_) / map_resolution_;
        goal_y_ = (msg->pose.position.y - map_origin_y_) / map_resolution_;
        
        PointCell goal;
        goal.x = goal_x_;
        goal.y = goal_y_;
        
        PointCell start;
        start.x = start_x_;
        start.y = start_y_;
        
        goal_.pose = msg->pose;
    }
    
    mutex.unlock();
    
    makePlan(start_, goal_, threhold_, plan_);
    
}
 
} /* end of namespace */
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sampling");
    
    rrt_planner::RRTPlanner planner;
    
    ros::spin();
    return 0;
};
