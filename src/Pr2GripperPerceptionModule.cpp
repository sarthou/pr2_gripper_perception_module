#include "pr2_gripper_perception_module/Pr2GripperPerceptionModule.h"

#include <numeric>

#include <pluginlib/class_list_macros.h>

#include "overworld/Utils/ShellDisplay.h"

namespace owds
{

  Pr2GripperPerceptionModule::Pr2GripperPerceptionModule() : PerceptionModuleRosBase("", true)
  // We need to wait the initialization to know the topic name
  {
    pr2_left_tip_world_id_ = -1;
    pr2_right_tip_world_id_ = -1;
    side_ = PR2_GRIPER_UNDEFINED;

    is_init_ = false;
    left_tip_pressure_ = 0;
    left_tip_pressure_prev_ = 0;
    right_tip_pressure_ = 0;
    right_tip_pressure_prev_ = 0;

    has_picked_ = false;
    prc_obj_id_ = 0;

    pressure_threshold_ = 800;
    distance_threshold_ = 0.03;
    min_period_ = 0.5;
  }

  void Pr2GripperPerceptionModule::setParameter(const std::string &parameter_name, const std::string &parameter_value)
  {
    if (parameter_name == "side")
      side_ = static_cast<Pr2GripperSide_e>(std::stoi(parameter_value));
    else if (parameter_name == "pressure_threshold")
      pressure_threshold_ = std::stod(parameter_value);
    else if (parameter_name == "distance_threshold")
      distance_threshold_ = std::stod(parameter_value);
    else if (parameter_name == "min_period")
      min_period_ = std::stod(parameter_value);
    else
      ShellDisplay::warning("[Pr2GripperPerceptionModule] Unkown parameter " + parameter_name);
  }

  bool Pr2GripperPerceptionModule::closeInitialization()
  {
    if (side_ == PR2_GRIPER_UNDEFINED)
    {
      ShellDisplay::error("[" + module_name_ + "] No gripper side has been defined");
      return false;
    }

    setTopicName("pressure/" + std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_motor");

    std::string left_finger_link = std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_l_finger_tip_link";
    std::string right_finger_link = std::string((side_ == PR2_GRIPPER_LEFT) ? "l" : "r") + "_gripper_r_finger_tip_link";

    pr2_left_tip_world_id_ = world_client_->getLinkId(robot_engine_id_, left_finger_link);
    if(pr2_left_tip_world_id_ == -1)
      throw std::runtime_error("[" + module_name_ + "] link " + left_finger_link + " does not exist");

    pr2_right_tip_world_id_ = world_client_->getLinkId(robot_engine_id_, right_finger_link);
    if(pr2_right_tip_world_id_ == -1)
      throw std::runtime_error("[" + module_name_ + "] link " + left_finger_link + " does not exist");

    return true;
  }

  bool Pr2GripperPerceptionModule::perceptionCallback(const pr2_msgs::PressureState &msg)
  {
    if ((ros::Time::now() - last_update_).toSec() < min_period_)
      return false;
    last_update_ = ros::Time::now();

    left_tip_pressure_prev_ = left_tip_pressure_;
    right_tip_pressure_prev_ = right_tip_pressure_;

    left_tip_pressure_ = 0;
    for (auto &p : msg.l_finger_tip)
      left_tip_pressure_ += (double)p;
    left_tip_pressure_ /= (double)msg.l_finger_tip.size();

    right_tip_pressure_ = 0;
    for (auto &p : msg.r_finger_tip)
      right_tip_pressure_ += (double)p;
    right_tip_pressure_ /= (double)msg.r_finger_tip.size();

    if (is_init_)
    {
      double pressure_diff_left = left_tip_pressure_ - left_tip_pressure_prev_;
      double pressure_diff_right = right_tip_pressure_ - right_tip_pressure_prev_;

      if (has_picked_ == false)
      {
        if ((pressure_diff_left > pressure_threshold_) && (pressure_diff_right > pressure_threshold_))
        {
          double dist = getGripperDistance();
          if (dist >= distance_threshold_)
          {
            if (side_ == PR2_GRIPPER_LEFT)
              current_prc_obj_id_ = "prc_obj_pr2_gripper_left_" + std::to_string(prc_obj_id_);
            else
              current_prc_obj_id_ = "prc_obj_pr2_gripper_right_" + std::to_string(prc_obj_id_);

            Percept<Object> percept(current_prc_obj_id_, false);
            Shape_t shape;
            shape.type = SHAPE_CUBE;
            shape.scale.fill(dist);
            percept.setShape(shape);
            // we can not take the mass from the ontology since the percept object does not exist in (fake id)
            percept.setSeen();
            percepts_.insert(std::make_pair(percept.id(), percept));

            Hand* hand = nullptr;

            if (side_ == PR2_GRIPPER_LEFT)
            {
              if (robot_agent_->getLeftHand() != nullptr)
                hand = robot_agent_->getLeftHand();
            }
            else
            {
              if (robot_agent_->getRightHand() != nullptr)
                hand = robot_agent_->getRightHand();
            }

            if (hand != nullptr)
            {
              std::cout << "---------------------------------------------------------------> Pick Percept " << current_prc_obj_id_ << " with hand " << hand->id() << std::endl;
              percepts_.at(current_prc_obj_id_).setInHand(hand);
              percepts_.at(current_prc_obj_id_).updatePose(Pose()); // meaning the object is at the centrer of the hand
            }
            has_picked_ = true;

            return true;
          }
        }

        return false;
      }
      else
      {
        if ((pressure_diff_left < -pressure_threshold_) && (pressure_diff_right < -pressure_threshold_))
        {
          std::cout << "---------------------------------------------------------------> Release Percept " << current_prc_obj_id_ << std::endl;
          has_picked_ = false;
          percepts_.at(current_prc_obj_id_).setUnseen();
          auto hand = percepts_.at(current_prc_obj_id_).getHandIn();

          if (percepts_.at(current_prc_obj_id_).isInHand())
          {
            std::cout << "[Pr2GripperPerceptionModule] remove percept form hand " << current_prc_obj_id_ << std::endl;
            percepts_.at(current_prc_obj_id_).removeFromHand();
            std::cout << "[Pr2GripperPerceptionModule] is in hand = " << percepts_.at(current_prc_obj_id_).isInHand() << std::endl;
          }

          percepts_.at(current_prc_obj_id_).unsetPose();

          current_prc_obj_id_ = "";
          prc_obj_id_++;
        }
        else
        {
          if (side_ == PR2_GRIPPER_LEFT)
          {
            if (robot_agent_->getLeftHand() != nullptr)
              percepts_.at(percepts_.at(current_prc_obj_id_).id()).updatePose(Pose()); // meaning the object is at the centrer of the hand
          }
          else if (robot_agent_->getRightHand() != nullptr)
            percepts_.at(percepts_.at(current_prc_obj_id_).id()).updatePose(Pose()); // meaning the object is at the centrer of the hand
        }
        return true;
      }
      return true;
    }
    else
    {
      is_init_ = true;
      return false;
    }
  }

  double Pr2GripperPerceptionModule::getGripperDistance()
  {
    auto left_tip_link_state = world_client_->getBasePositionAndOrientation(pr2_left_tip_world_id_);
    Pose left_tip_pose(left_tip_link_state.first, left_tip_link_state.second);

    auto right_tip_link_state = world_client_->getBasePositionAndOrientation(pr2_right_tip_world_id_);
    Pose right_tip_pose(right_tip_link_state.first, right_tip_link_state.second);

    return left_tip_pose.distanceTo(right_tip_pose) - 0.0275; // 0.0275 correspond to grippers offset
  }

} // namespace owds

PLUGINLIB_EXPORT_CLASS(owds::Pr2GripperPerceptionModule, owds::PerceptionModuleBase_<owds::Object>)