#include <controller_interface/controller.h>
#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <diff_drive_controller/odometry.h>
#include <diff_drive_controller/speed_limiter.h>

namespace diff_drive_controller{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in
   * the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder or sphere in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie
   *  within the contact patch
   */
  class DiffDriveController
      : public
controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    DiffDriveController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new
     * velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    /// Hardware handles:
    std::vector<hardware_interface::JointHandle> left_wheel_joints_;
    std::vector<hardware_interface::JointHandle> right_wheel_joints_;

    /// Velocity command related:
    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    /// Publish executed commands
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>
> cmd_vel_pub_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> >
odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> >
tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_multiplier_;
    double wheel_radius_multiplier_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

    /// Publish limited velocity:
    bool publish_cmd_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::vector<std::string>& wheel_names);

    /**
     * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and
     * separation
     * \param root_nh Root node handle
     * \param left_wheel_name Name of the left wheel joint
     * \param right_wheel_name Name of the right wheel joint
     */
    bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                               const std::string& left_wheel_name,
                               const std::string& right_wheel_name,
                               bool lookup_wheel_separation,
                               bool lookup_wheel_radius);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle&
controller_nh);

  };

  PLUGINLIB_EXPORT_CLASS(diff_drive_controller::DiffDriveController,
controller_interface::ControllerBase);
} // namespace diff_drive_controller