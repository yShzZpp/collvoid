//
// Created by danielclaes on 22/06/15.
//

#include "collvoid_local_planner/collvoid_scoring_function.h"
#include "cti_spdlog.h"
#include "spdlog/spdlog.h"

namespace collvoid_scoring_function
{

  void CollvoidScoringFunction::init(ros::NodeHandle nh)
  {
    get_me_srv_ = nh.serviceClient<collvoid_srvs::GetMe>("get_me");                      // 获取自身的信息：id，半径，是否全向移动，是否可控制，位置，速度，footprint
    get_neighbors_srv_ = nh.serviceClient<collvoid_srvs::GetNeighbors>("get_neighbors"); // 获取邻居的信息：id，半径，是否全向移动，是否可控制，位置，速度，footprint
    neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1);      // 发布邻居的信息
    samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);          // 发布采样的信息
    vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);                         // 发布VO的信息

    ros::NodeHandle co_nh("~/collvoid");
    use_truncation_ = co_nh.param("use_truncation", true); // 是否使用截断
    trunc_time_ = co_nh.param("trunc_time", 8.);           // 10
    use_polygon_footprint_ = co_nh.param("use_polygon_footprint", true);
    max_dist_vo_ = co_nh.param("max_dist_vo", 0.1); // 1
    dist_agent_vo_scale_ = co_nh.param("dist_agent_vo_scale", 0.3);
    dist_human_vo_scale_ = co_nh.param("dist_hunman_vo_scale", 0.5);
    dist_static_vo_scale_ = co_nh.param("dist_static_vo_scale", 0.4);
    same_direction_scale_ = co_nh.param("same_direction_scale", 0.4);
    early_time_ = co_nh.param("early_time", 0.0); // 计算平均速度提早的时间,预计多的速度
    points_.clear();
    ROS_INFO("Collvoid Scoring init done! Trunctime %f, max_dist_vo %f", trunc_time_, max_dist_vo_);
    SPDLOG_INFO("Collvoid Scoring init done! name:[{}], trunctime: [{}], max_dist_vo: [{}]", co_nh.getNamespace(), trunc_time_, max_dist_vo_);
    SPDLOG_INFO("Collvoid Scoring init done! same_direction_scale: {}", same_direction_scale_);
    // holo_robot_ = false;
  }

  bool CollvoidScoringFunction::getMe()
  {
    collvoid_srvs::GetMe srv;
    if (get_me_srv_.call(srv))
    {
      me_ = createAgentFromMsg(srv.response.me);
      me_->use_truncation_ = use_truncation_;
      me_->trunc_time_ = trunc_time_;
      me_->use_polygon_footprint_ = use_polygon_footprint_;
      me_->type_vo_ = HRVOS;
      // SPDLOG_INFO("angle z :{}",me_->angle_z_);

      // ROS_INFO("GOT ME");
      //  SPDLOG_INFO("CDWA scoring: Got me");
      return true;
    }
    else
    {
      ROS_INFO("Collvoid Scoring: Could not get me");
      SPDLOG_INFO("CDWA scoring: Could not get me");
      return false;
    }
  }

  bool CollvoidScoringFunction::getNeighbors()
  {
    collvoid_srvs::GetNeighbors srv;
    if (get_neighbors_srv_.call(srv))
    {
      for (collvoid_msgs::PoseTwistWithCovariance msg : srv.response.neighbors)
      {
        me_->agent_neighbors_.push_back(createAgentFromMsg(msg));
      }

      std::sort(me_->agent_neighbors_.begin(), me_->agent_neighbors_.end(),
                boost::bind(&CollvoidScoringFunction::compareNeighborsPositions, this, _1, _2));
      collvoid::publishNeighborPositionsBare(me_->agent_neighbors_, "/map", "/map", neighbors_pub_);
      // SPDLOG_INFO("CDWA scoring: Got neighbors");
      return true;
    }
    else
    {
      ROS_INFO("Collvoid Scoring: Could not get nieghbors");
      SPDLOG_INFO("CDWA scoring: Could not get neighbors");
      return false;
    }
  }

  bool CollvoidScoringFunction::compareNeighborsPositions(const AgentPtr& agent1, const AgentPtr& agent2)
  {
    return compareVectorPosition(agent1->position_, agent2->position_);
  }

  bool CollvoidScoringFunction::compareVectorPosition(const collvoid::Vector2& v1, const collvoid::Vector2& v2)
  {
    return collvoid::absSqr(me_->position_ - v1) <= collvoid::absSqr(me_->position_ - v2);
  }

  AgentPtr CollvoidScoringFunction::createAgentFromMsg(collvoid_msgs::PoseTwistWithCovariance& msg)
  {
    AgentPtr agent = AgentPtr(new Agent());
    agent->radius_ = msg.radius;
    agent->controlled_ = msg.controlled;
    agent->position_ = Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
    agent->heading_ = tf::getYaw(msg.pose.pose.orientation);

    std::vector<Vector2> minkowski_footprint;
    for (geometry_msgs::Point32 p : msg.footprint.polygon.points)
    {
      minkowski_footprint.push_back(Vector2(p.x, p.y));
    }
    agent->footprint_ = rotateFootprint(minkowski_footprint, agent->heading_);

    if (msg.holo_robot)
    {
      agent->velocity_ = rotateVectorByAngle(msg.twist.twist.linear.x,
                                             msg.twist.twist.linear.y, agent->heading_);
    }
    else
    {
      double dif_x, dif_y, dif_ang, time_dif;
      time_dif = 0.1;
      dif_ang = time_dif * msg.twist.twist.angular.z;
      dif_x = msg.twist.twist.linear.x * cos(dif_ang / 2.0);
      dif_y = msg.twist.twist.linear.x * sin(dif_ang / 2.0);
      agent->velocity_ = rotateVectorByAngle(dif_x, dif_y, agent->heading_);
      agent->angle_z_ = dif_ang;
    }

    return agent;
  }

  bool CollvoidScoringFunction::prepare()
  {
    // Get me
    if (!getMe())
    {
      return false;
    }

    // Get neighbors
    if (!getNeighbors())
    {
      return false;
    }
    // Calculate VOs
    me_->computeAgentVOs();

    collvoid::publishVOs(me_->position_, me_->all_vos_, use_truncation_, "/map", "/map", vo_pub_);
    collvoid::publishPoints(me_->position_, points_, "/map", "/map", samples_pub_);
    // for (size_t i = 0; i < me_->all_vos_.size(); ++i) {
    //     VO v = me_->all_vos_.at(i);
    //     ROS_INFO("Origin %f %f", v.point.x(), v.point.y()) ;
    //  }

    points_.clear();

    // Add constraints - Not necessary due to sampling?
    // SPDLOG_INFO("CDWA scoring: Prepared");
    return true;
  }

  double CollvoidScoringFunction::scoreTrajectory(Trajectory& traj)
  {
    if (traj.getPointsSize() < 1)
      return 0;

    // TODO: check if goalHeading and endPoint are in the same reference frame
    double x_s, y_s, th_s;                                   // 起点位姿态
    double x_h, y_h, th_h;                                   // 中点
    double x_e, y_e, th_e;                                   // 终点
    traj.getPoint(0, x_s, y_s, th_s);                        // 起点
    traj.getPoint(traj.getPointsSize() / 2, x_h, y_h, th_h); // 中点
    traj.getEndpoint(x_e, y_e, th_e);                        // 终点

    // ROS_INFO("start orientation / end %f, %f", th_s,th);

    double time_diff = (int)traj.getPointsSize() * traj.time_delta_; // 步数*每一步的时间 = 采样点的总时间
    time_diff = time_diff - 0.3 > 0 ? time_diff - 0.3 : time_diff;
    double dx, dy, dtheta;
    double vel_x, vel_y, vel_theta;
    double c_vel_x = me_->getVelocity().x(), c_vel_theta = me_->getVelocity().y();
    // vel_x = traj.xv_;
    // vel_y = traj.yv_;
    // vel_theta = traj.thetav_;
    dx = x_e - x_s; // 位置差
    dy = y_e - y_s;
    dtheta = th_e - th_s;

    Vector2 test_vel = Vector2();
    /*if (fabs(vel_y) == 0.) {
        double dif_x, dif_y, dif_ang, time_dif;
        time_dif = 5 * traj.time_delta_;
        dif_ang = 2 * time_dif * vel_theta;
        dif_x = time_dif * vel_x * cos(dif_ang / 2.0);
        dif_y = time_dif * vel_x * sin(dif_ang / 2.0);
        test_vel = rotateVectorByAngle(dif_x, dif_y, me_->heading_);
    }
    else {
        test_vel = rotateVectorByAngle(vel_x, vel_y, me_->heading_);
    }*/
    // test_vel = rotateVectorByAngle(vel_x, vel_y, -th_s + me_->heading_ + vel_theta / 3.) / time_diff;
    test_vel = rotateVectorByAngle(dx, dy, -th_s + me_->heading_ + dtheta / 3.) / time_diff; // 平均速度
    // test_vel = Vector2(vel_x,vel_y);
    // test_vel = Vector2(x_h, y_h);

    double cost = calculateVelCosts(test_vel, me_->agent_vos_, me_->use_truncation_);

    if (cost > 0.)
    { // vo内有轨迹
      int n = (int)me_->agent_vos_.size();
      if (cost >= n * 2)
        return -1;
      cost = 1 + (n * (n + 1) / 2.) * 2. / cost;
    }

    else
    { // vo内无轨迹// (最远距离阈值 - 离最近的agentvo平方根)/最远距离阈值 （必定为非负数）
      cost = dist_agent_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->agent_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
      cost += dist_human_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->human_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
      cost += dist_static_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->static_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
      cost += same_direction_scale_ * (dtheta > 0);
      if (cost < 0.)
      {
        SPDLOG_INFO("scoring {} is {}", th_s, cost);
        return -1;
      }
      // cost += same_direction_scale_ * (me_->angle_z_ * th_s >= 0 ? 0 : 1); //同向为0 反向为1
      // cost = std::max(std::min(cost + 0.3 * dtheta / 2, cost), 0.);
      // cost += 1 * (angle < 0 ? -1 : 1);
      // 速度取大的
      // cost += 1.0 * (c_vel_x - test_vel.x());
    }
    VelocitySample v;
    v.velocity = test_vel;
    v.cost = cost;
    points_.push_back(v);

    // ROS_INFO("Collvoid Scoring costs: %f for vector %f, %f, speed %f, ang %f time dif %f", cost, test_vel.x(), test_vel.y(), vel_x, vel_theta, traj.time_delta_);

    // traj.x

    return cost;
  }
}
