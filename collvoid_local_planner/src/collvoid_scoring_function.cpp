//
// Created by danielclaes on 22/06/15.
//

#include "collvoid_local_planner/collvoid_scoring_function.h"
#include "cti_spdlog.h"
#include "geometry_msgs/PoseArray.h"
#include "spdlog/spdlog.h"

namespace collvoid_scoring_function
{

  void CollvoidScoringFunction::init(ros::NodeHandle nh)
  {
    get_me_srv_ = nh.serviceClient<collvoid_srvs::GetMe>("get_me");                      // 获取自身的信息：id，半径，是否全向移动，是否可控制，位置，速度，footprint
    get_neighbors_srv_ = nh.serviceClient<collvoid_srvs::GetNeighbors>("get_neighbors"); // 获取邻居的信息：id，半径，是否全向移动，是否可控制，位置，速度，footprint
    neighbors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("neighbors", 1);      // 发布邻居的信息
    samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);          // 发布采样的信息
    mink_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("mink", 1);                  // mink
    center_pub_ = nh.advertise<geometry_msgs::PoseArray>("center", 1);                   // mink
    vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);                         // 发布VO的信息

    id_ = nh.param<std::string>("/robot_attribute/number", "robot");
    use_sim_time_ = nh.param<bool>("/use_sim_time", false);
    if (!nh.getParam("/robot_attribute/number", id_) && use_sim_time_)
    {
      id_ = nh.getNamespace();
      if (strcmp(id_.c_str(), "/") == 0)
      {
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        id_ = std::string(hostname);
      }
      // remove funky "/" to get uniform name in python and here
      id_.erase(std::remove(id_.begin(), id_.end(), '/'), id_.end());
    }
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
    SPDLOG_INFO("Collvoid Scoring init done! same_direction_scale: [{}], use_sim_time: [{}]", same_direction_scale_, use_sim_time_);
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
    agent->id_ = msg.robot_id;
    agent->radius_ = msg.radius;
    agent->controlled_ = msg.controlled;
    agent->position_ = Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
    agent->heading_ = tf::getYaw(msg.pose.pose.orientation);
    double angle_between_me = agent->heading_;

    std::vector<Vector2> minkowski_footprint;
    for (geometry_msgs::Point32 p : msg.footprint.polygon.points)
    {
      minkowski_footprint.push_back(Vector2(p.x, p.y));
    }
    if (me_ && me_->id_ != agent->id_)
    {
      angle_between_me = agent->heading_ - me_->heading_;
      agent->footprint_ = rotateFootprint(minkowski_footprint, angle_between_me);
    }
    else
    {
      agent->footprint_ = rotateFootprint(minkowski_footprint, agent->heading_);
    }
    // agent->footprint_ = minkowski_footprint;

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
      agent->angle_between_me_ = angle_between_me;
      agent->anglez_ = dif_ang;
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

    if (use_sim_time_ && me_->agent_neighbors_.size() > 0)
    {
      geometry_msgs::PolygonStamped mink;
      mink.header.frame_id = id_ + "/base_link";
      auto mink_sum = minkowskiSum(me_->footprint_, me_->agent_neighbors_.front()->footprint_);
      // for (auto m : mink_sum)
      for (auto m : mink_sum)
      {
        geometry_msgs::Point32 point;
        point.x = m.x();
        point.y = m.y();
        mink.polygon.points.push_back(point);
      }
      mink_pub_.publish(mink);
      if (me_->agent_vos_.size() > 0 || me_->static_vos_.size() > 0)
      {
        VO vo = me_->agent_vos_.size() > 0 ? me_->agent_vos_.front() : me_->static_vos_.front();
        geometry_msgs::PoseArray poses;
        geometry_msgs::Pose pose;
        poses.header.frame_id = id_ + "/base_link";

        pose.position.x = vo.left_leg_dir.x();
        pose.position.y = vo.left_leg_dir.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.trunc_left.x();
        pose.position.y = vo.trunc_left.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.trunc_line_center.x();
        pose.position.y = vo.trunc_line_center.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.point.x();
        pose.position.y = vo.point.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.trunc_right.x();
        pose.position.y = vo.trunc_right.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.right_leg_dir.x();
        pose.position.y = vo.right_leg_dir.y();
        poses.poses.push_back(pose);

        pose.position.x = vo.relative_position.x();
        pose.position.y = vo.relative_position.y();
        poses.poses.push_back(pose);

        center_pub_.publish(poses);
      }
    }
    // for (size_t i = 0; i < me_->all_vos_.size(); ++i) {
    //     VO v = me_->all_vos_.at(i);
    //     ROS_INFO("Origin %f %f", v.point.x(), v.point.y()) ;
    //  }

    points_.clear();

    // Add constraints - Not necessary due to sampling?
    // SPDLOG_INFO("CDWA scoring: Prepared");
    return true;
  }

  // 归一化角度到[0, 2*PI]范围内
  double normalizeAngle(double angle)
  {
    while (angle < 0)
      angle += 2 * M_PI;
    while (angle >= 2 * M_PI)
      angle -= 2 * M_PI;
    return angle;
  }

  // 计算角度差
  double calculateAngleDifference(double angle1, double angle2)
  {
    double diff = std::fabs(normalizeAngle(angle1) - normalizeAngle(angle2));
    if (diff > M_PI)
      diff = 2 * M_PI - diff;
    return diff;
  }

  // 计算朝向的成本
  double calculateAngleCosts(const double heading1, const double heading2, const double anglez1, const double anglez2)
  {
    double cost = 1.0;

    // 计算两个机器人头朝向的角度差 (map坐标系下)
    double headingDiff = calculateAngleDifference(heading1, heading2);

    // 计算两个机器人相对于各自坐标系的旋转方向
    bool bothLeftOrRight = (anglez1 >= 0 && anglez2 >= 0) || (anglez1 < 0 && anglez2 < 0);
    bool oneLeftOneRight = (anglez1 >= 0 && anglez2 < 0) || (anglez1 < 0 && anglez2 >= 0);

    // 同向：角度差30度以内
    if (headingDiff < M_PI / 6)
    { // 30 degrees in radians
      cost = 0.0;
    }
    // 相交：角度差30度到90度之间
    else if (headingDiff >= M_PI / 6 && headingDiff <= M_PI / 2)
    { // 30 to 90 degrees in radians
      if (oneLeftOneRight)
      {
        cost = 0.0;
      }
    }
    // 反向：角度差90度到180度之间
    else if (headingDiff > M_PI / 2 && headingDiff <= M_PI)
    { // 90 to 180 degrees in radians
      if (bothLeftOrRight)
      {
        cost = 0.0;
      }
    }

    return cost;
  }

  // 获取自身的footprint
  std::vector<geometry_msgs::Point> CollvoidScoringFunction::getMeFootprint()
  {
    std::vector<geometry_msgs::Point> footprint;
    if (!me_)
    {
      return footprint;
    }
    for (const auto& p : me_->footprint_)
    {
      // ROS_INFO("%s: ",me_->id_.c_str(), );
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      footprint.push_back(point);
    }
    return footprint;
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
    // test_vel = rotateVectorByAngle(dx, dy, -th_s + me_->heading_ + dtheta / 3.); // 平均速度
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
    {
      cost = 0;
      int move_agent = 0;
      for (const auto agent : me_->agent_neighbors_)
      {
        // 没有移动或距离小于2米不考虑
        if (agent->getVelocity().x() < EPSILON || abs(agent->getPosition() - me_->getPosition()) < 2)
        {
          continue;
        }
        move_agent++;
        cost += calculateAngleCosts(me_->heading_, agent->heading_, dtheta, agent->anglez_);
      }
      if (move_agent > 0)
      {
        cost = cost / move_agent * same_direction_scale_;
      }
      // vo内无轨迹// (最远距离阈值 - 离最近的agentvo平方根)/最远距离阈值 （必定为非负数）
      cost += dist_agent_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->agent_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
      cost += dist_human_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->human_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
      cost += dist_static_vo_scale_ * std::max((max_dist_vo_ - sqrt(std::max(minDistToVOs(me_->static_vos_, test_vel, use_truncation_, true), 0.))) / max_dist_vo_, 0.);
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
