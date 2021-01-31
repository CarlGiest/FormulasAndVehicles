#include <gazebo_range_sensor_plugin.h>
#include <math.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(RangesPlugin)

RangesPlugin::RangesPlugin() : ModelPlugin() {}

RangesPlugin::~RangesPlugin() { update_connection_->~Connection(); }

void RangesPlugin::getSdfParams(sdf::ElementPtr sdf) {
  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<double>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[ranges_plugin] Using default publication rate of " << pub_rate_
           << "Hz\n";
  }
  if (sdf->HasElement("rangesTopic")) {
    ranges_topic_ = sdf->GetElement("rangesTopic")->Get<std::string>();
  } else {
    ranges_topic_ = kDefaultRangesTopic;
  }
  if (sdf->HasElement("rangeNoiseStd")) {
    range_noise_std_ = sdf->GetElement("rangeNoiseStd")->Get<double>();
  } else {
    range_noise_std_ = kDefaultRangesNoise;
    gzwarn << "[ranges_plugin] Using default noise " << kDefaultRangesNoise
           << "\n";
  }
  if (sdf->HasElement("fovCamera")) {
    // we'll check visibility using half the fov angle
    max_fov_angle_ =
        sdf->GetElement("fovCamera")->Get<double>() / 2.0 * (M_PI / 180.0);
  } else {
    max_fov_angle_ = (kDefaultFov / 2.0) * (M_PI / 180.0);
    gzwarn << "[ranges_plugin] Using default field of view angle "
           << kDefaultFov << "\n";
  }
  if (sdf->HasElement("viewingAngle")) {
    // we'll check visibility using half the viewing angle
    max_viewing_angle_ =
        sdf->GetElement("viewingAngle")->Get<double>() / 2.0 * (M_PI / 180.0);
  } else {
    max_viewing_angle_ = (kDefaultViewingAngle / 2.0) * (M_PI / 180.0);
    gzwarn << "[ranges_plugin] Using default viewing angle "
           << kDefaultViewingAngle << "\n";
  }
  if (sdf->HasElement("dropProb")) {
    drop_prob_ = sdf->GetElement("dropProb")->Get<double>();
  } else {
    drop_prob_ = kDefaultDropProb;
    gzwarn << "[ranges_plugin] Using default probability " << kDefaultDropProb
           << " for dropping measurements\n";
  }
  if (sdf->HasElement("maxDetectionDist")) {
    max_detection_dist_ = sdf->GetElement("maxDetectionDist")->Get<double>();
  } else {
    max_detection_dist_ = kDefaultMaxDetectionDist;
    gzwarn << "[ranges_plugin] Using default max detection distance "
           << kDefaultMaxDetectionDist << "\n";
  }
  if (sdf->HasElement("maxDetectionDist")) {
    dist_drop_prob_exponent_ =
        sdf->GetElement("maxDetectionDist")->Get<double>();
  } else {
    dist_drop_prob_exponent_ = kDefaultDistDropProbExponent;
    gzwarn << "[ranges_plugin] Using default Exponent "
           << kDefaultDistDropProbExponent
           << " for probability that too far away measurements are dropped \n";
  }
}

void RangesPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  getSdfParams(sdf);
  model_ = model;
  world_ = model_->GetWorld();
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("ROS node for gazebo not initialized");
    return;
  }
  node_handle_ = new ros::NodeHandle(namespace_);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RangesPlugin::OnUpdate, this, _1));

  ranges_pub_ = node_handle_->advertise<range_sensor::RangeMeasurementArray>(
      ranges_topic_, 1);

  initialized_ = false;
  tag_axis_ = ignition::math::Vector3d(0.0, 1.0, 0.0);
}

void RangesPlugin::OnUpdate(const common::UpdateInfo &) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_pub_time_).Double();

  if (!initialized_) {
    auto model = world_->ModelByName("ring");
    if (model && model->GetChildLink("tag_1_link")) {
      pos_tag_1_ = world_->ModelByName("ring")
                       ->GetChildLink("tag_1_link")
                       ->RelativePose()
                       .Pos();
      gzmsg << "[ranges plugin] Tag 1 Position found.\n";
      gzmsg << "[ranges_plugin] Pos Tag 1 " << pos_tag_1_
           << " \n";
    }
    if (model && model->GetChildLink("tag_2_link")) {
      pos_tag_2_ = world_->ModelByName("ring")
                       ->GetChildLink("tag_2_link")
                       ->RelativePose()
                       .Pos();
      gzmsg << "[ranges plugin] Tag 2 Position found.\n";
    }
    if (model && model->GetChildLink("tag_3_link")) {
      pos_tag_3_ = world_->ModelByName("ring")
                       ->GetChildLink("tag_3_link")
                       ->RelativePose()
                       .Pos();
      gzmsg << "[ranges plugin] Tag 3 Position found.\n";
    }
    if (model && model->GetChildLink("tag_4_link")) {
      pos_tag_4_ = world_->ModelByName("ring")
                       ->GetChildLink("tag_4_link")
                       ->RelativePose()
                       .Pos();
      gzmsg << "[ranges plugin] Tag 4 Position found.\n";
      initialized_ = true;
    }
    // Generate Grid for floor AprilTags
    for(double j=0.0; j<9.0 ; ++j){
      for(double i=0.0; i<7.0; ++i){
        auto tmp_tag = ignition::math::Vector3d(1.56-i*0.25, 0.06+j*0.39375, -1.3);
        floor_tags_.push_back(tmp_tag);
        //gzmsg << "[ranges plugin] Tag " << i+j*7.0 << " pos at " << tmp_tag << "\n";
      }
    }
  }

  if ((dt > 1.0 / pub_rate_) && (initialized_)) {
    range_sensor::RangeMeasurementArray msg_array;
    msg_array.measurements.clear();
    msg_array.header.stamp = ros::Time::now();
    msg_array.header.frame_id = "map";

    // get world pose
    ignition::math::Vector3d pos_sensor =
        model_->GetLink("range_sensor_link")->WorldPose().Pos();
    // gzmsg << "[ranges_plugin] Pos Tag 1 " << pos_sensor << " \n";
    // get orientation of body x-axis
    ignition::math::Vector3d x_unit_vector(1.0, 0.0, 0.0);
    ignition::math::Vector3d body_x_axis = model_->GetLink("range_sensor_link")
                                               ->WorldPose()
                                               .Rot()
                                               .RotateVector(x_unit_vector);
    ignition::math::Vector3d y_unit_vector(0.0, 1.0, 0.0);
    ignition::math::Vector3d body_y_axis = model_->GetLink("range_sensor_link")
                                               ->WorldPose()
                                               .Rot()
                                               .RotateVector(y_unit_vector);
    // ignition::math::Vector3d pos_ring =
    //               model_->GetLink("ring::base_link")->WorldPose().Pos();
    auto model_ring = world_->ModelByName("ring");
    ignition::math::Vector3d pos_ring;
    if(model_ring && model_ring->GetChildLink("base_link")){
      pos_ring = world_->ModelByName("ring")
                       ->GetChildLink("base_link")
                       ->WorldPose()
                       .Pos();
      //gzmsg << "[ranges_plugin] Ring" << pos_ring << "\n";
    }
    auto pos_tag_1_abs =  pos_ring + pos_tag_1_;
    auto pos_tag_2_abs =  pos_ring + pos_tag_2_;
    auto pos_tag_3_abs =  pos_ring + pos_tag_3_;
    auto pos_tag_4_abs =  pos_ring + pos_tag_4_;
    // tag 1
    ignition::math::Vector3d sensor_to_tag_1 = pos_tag_1_abs - pos_sensor;
    if (IsDetected(sensor_to_tag_1, body_x_axis)) {
      range_sensor::RangeMeasurement msg = GetRangeMsg(1, sensor_to_tag_1);
      msg_array.measurements.push_back(msg);
    }

    // tag 2
    ignition::math::Vector3d sensor_to_tag_2 = pos_tag_2_abs - pos_sensor;
    if (IsDetected(sensor_to_tag_2, body_x_axis)) {
      range_sensor::RangeMeasurement msg = GetRangeMsg(2, sensor_to_tag_2);
      msg_array.measurements.push_back(msg);
    }

    // tag 3
    ignition::math::Vector3d sensor_to_tag_3 = pos_tag_3_abs - pos_sensor;
    if (IsDetected(sensor_to_tag_3, body_x_axis)) {
      range_sensor::RangeMeasurement msg = GetRangeMsg(3, sensor_to_tag_3);
      msg_array.measurements.push_back(msg);
    }

    // tag 4
    ignition::math::Vector3d sensor_to_tag_4 = pos_tag_4_abs - pos_sensor;
    if (IsDetected(sensor_to_tag_4, body_x_axis)) {
      range_sensor::RangeMeasurement msg = GetRangeMsg(4, sensor_to_tag_4);
      msg_array.measurements.push_back(msg);
    }
    for(int i=0; i<63; ++i){
      ignition::math::Vector3d tmp_to_tag = floor_tags_.at(i) - pos_sensor;
      if (IsDetected(tmp_to_tag, body_x_axis, body_y_axis)) {
        range_sensor::RangeMeasurement msg = GetRangeMsg(i+5, tmp_to_tag);
        msg_array.measurements.push_back(msg);
      }
    }

    ranges_pub_.publish(msg_array);
    last_pub_time_ = current_time;
  }
}

bool RangesPlugin::IsDetected(ignition::math::Vector3d sensor_to_tag,
                              ignition::math::Vector3d body_x_axis) {
  // tag might not be visible, determine whether tag is in fov of camera
  double fov_angle = acos(sensor_to_tag.Dot(body_x_axis) /
                          (sensor_to_tag.Length() * body_x_axis.Length()));

  // camera might not be facing tag from front
  double viewing_angle = acos(tag_axis_.Dot(body_x_axis) /
                              (tag_axis_.Length() * body_x_axis.Length()));

  bool is_visible =
      (fov_angle < max_fov_angle_) && (viewing_angle < max_viewing_angle_);

  // measurement might be dropped for whatever reason
  double p = uniform_real_distribution_(random_generator_);
  // additional drop probability that increases with distance to tag
  double p_dist = uniform_real_distribution_(random_generator_);
  double drop_prob_dist = GetDistanceDropProp(sensor_to_tag.Length());

  bool is_not_dropped = (p > drop_prob_) && (p_dist > drop_prob_dist);
  
  // return is_visible && is_not_dropped;
  return true;
}

bool RangesPlugin::IsDetected(ignition::math::Vector3d sensor_to_tag,
                              ignition::math::Vector3d body_x_axis,
                              ignition::math::Vector3d body_y_axis ) {
  // determine fov angle for both x and y
  double fov_angle_x = acos(sensor_to_tag.Dot(body_x_axis) /
                            (sensor_to_tag.Length() * body_x_axis.Length()));
  double fov_angle_y = acos(sensor_to_tag.Dot(body_y_axis) /
                            (sensor_to_tag.Length() * body_y_axis.Length()));                         
  double viewing_angle_x = acos(tag_axis_.Dot(body_x_axis) /
                                (tag_axis_.Length() * body_x_axis.Length()));
  double viewing_angle_y = acos(tag_axis_.Dot(body_y_axis) /
                                (tag_axis_.Length() * body_y_axis.Length()));
  bool is_visible =
        (fov_angle_x < max_fov_angle_) && (viewing_angle_x < max_viewing_angle_) &&
        (fov_angle_y < max_fov_angle_) && (viewing_angle_y < max_viewing_angle_);
  // measurement might be dropped for whatever reason
  double p = uniform_real_distribution_(random_generator_);
  // additional drop probability that increases with distance to tag
  double p_dist = uniform_real_distribution_(random_generator_);
  double drop_prob_dist = GetDistanceDropProp(sensor_to_tag.Length());

  bool is_not_dropped = (p > drop_prob_) && (p_dist > drop_prob_dist);

  // return is_visible && is_not_dropped;
  return true;
}

range_sensor::RangeMeasurement RangesPlugin::GetRangeMsg(
    int id, ignition::math::Vector3d sensor_to_tag) {
  range_sensor::RangeMeasurement msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.id = id;

  double distance = sensor_to_tag.Length();
  // add noise
  double noise =
      standard_normal_distribution_(random_generator_) * range_noise_std_;
  msg.range = distance + noise;
  return msg;
}

double RangesPlugin::GetDistanceDropProp(double dist) {
  double p = (1.0 / pow(max_detection_dist_, dist_drop_prob_exponent_)) *
             pow(dist, dist_drop_prob_exponent_);
  if (p > 1.0) {
    p = 1.0;
  }
  return p;
}

}  // namespace gazebo
