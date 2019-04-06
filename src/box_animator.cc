#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sstream>
#include <stdio.h>

namespace gazebo {
class AnimatedBox : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;

    double interval = 0;
    if (_sdf->HasElement("time_interval"))
      interval = _sdf->Get<double>("time_interval");

    double height = 0;
    if (_sdf->HasElement("height"))
      height = _sdf->Get<double>("height");
    double pitch = 0;
    if (_sdf->HasElement("pitch"))
      pitch = _sdf->Get<double>("pitch");
    double roll = 0;
    if (_sdf->HasElement("roll"))
      roll = _sdf->Get<double>("roll");

    int num_positions = 0;
    if (_sdf->HasElement("num_positions"))
      num_positions = _sdf->Get<double>("num_positions");

    std::stringstream ss;
    if (_sdf->HasElement("positions"))
      ss.str(_sdf->Get<std::string>("positions"));

    // create the animation
    gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation(
        "test", interval * num_positions, true));

    gazebo::common::PoseKeyFrame *key;

    double timestamp = 0;
    double x, y, yaw;
    while (!ss.eof()) {
      ss >> x >> y >> yaw;
      key = anim->CreateKeyFrame(timestamp);
      timestamp += interval;
      key->Translation(ignition::math::Vector3d(x, y, height));
      key->Rotation(ignition::math::Quaterniond(roll, pitch, yaw));
    }

    // set the animation
    _parent->SetAnimation(anim);
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
} // namespace gazebo