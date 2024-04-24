#include "ros/ros.h"
#include <vector>
#include <string>
#include <map>
#include <geometry_msgs/PoseStamde.h>q                                                                                                                



class RRTPlanner
{
public:
  RRTPlanner();
  ~RRTPlanner();
 
private:
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
  bool publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);
 
  TreeNode* root_;//这是整棵树的根部，即起点位置
  TreeNode* goal_node_;//终点位置
  bool buildRRT();
  bool drawSample();
  bool extendNearestNode(PointCell random);
  bool extendSingleStep(const TreeNode* rrtnode, TreeNode* &node, const PointCell random);
  void probSample();
  void addGoalOrientNode();
  void addRandomNode();
  bool goalReached(const TreeNode* nearestNode);
 
  void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
 
  /* map information */
  double map_resolution_;
  int map_sizex_;
  int map_sizey_;
  double map_origin_x_, map_origin_y_;
  std::vector<int> map_info_;
  std::string frame_;
  std::map<int, PointCell> freespace_;//在这里，我用一个map来表示栅格地图中没有障碍物的部分，我们可以只在空闲位置随机撒点试试看
 
  /* rrt elements */
  double threhold_, probability_;//判断到终点的容忍度，以及向终点方向拉扯的撒点概率
  /* random number generator */
  std::default_random_engine gen_;
  std::random_device rd_;//随机数生成器
  /* sampling parameters */
  int iteration_;//最大迭代次数
  int sample_num_;//每次随机撒点的数量
  double extend_step_;//树枝扩展的步长
  std::vector<TreeNode*> rrt_nodes_;
  std::vector<PointCell> current_sampling_;//本次撒点的样本
  
  geometry_msgs::PoseStamped start_;
  int start_x_;
  int start_y_;
  geometry_msgs::PoseStamped goal_;
  int goal_x_;
  int goal_y_;
  std::vector<geometry_msgs::PoseStamped> plan_;
 
  /* ros publisher */
  ros::Publisher plan_pub_, tree_pub_;
  ros::Subscriber map_sub_, pose_sub_, goal_sub_;
 
  /* mutex */
  boost::mutex mutex;
};
