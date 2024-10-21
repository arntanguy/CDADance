

#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

#include <mutex>
#include <thread>

​

    /**

     * @brief Describes data obtained by a subscriber, along with the time since
     it

     * was obtained

     *

     * @tparam Data Type of the data obtained by the subscriber

     */

    template<typename Data>

    struct SubscriberData

{
  bool isValid() const noexcept

  {
    return time_ <= maxTime_;
  }

  ​

      void
      operator=(const SubscriberData<Data> & data)

  {
    value_ = data.value_;

    time_ = data.time_;

    maxTime_ = data.maxTime_;
  }

  ​

      const Data &
      value() const noexcept

  {
    return value_;
  }

  ​

      void
      tick(double dt)

  {
    time_ += dt;
  }

  ​

      void
      maxTime(double t)

  {
    maxTime_ = t;
  }

  ​

      double
      time() const noexcept

  {
    return time_;
  }

  ​

      double
      maxTime() const noexcept

  {
    return maxTime_;
  }

  ​

      void
      value(const Data & data)

  {
    value_ = data;

    time_ = 0;
  }

​

    private :

    Data value_;

  double time_ = std::numeric_limits<double>::max();

  double maxTime_ = std::numeric_limits<double>::max();
};

​

    /**

     * @brief Simple interface for subscribing to data. It is assumed here that
     the

     * data will be acquired in a separate thread (e.g ROS spinner) and

     * setting/getting the data is thread-safe.

     * Don't forget to call tick(double dt) to update the time and value

     */

    template<typename Data>

    struct Subscriber

{
  /** Update time and value */

  void tick(double dt)

  {
    data_.tick(dt);
  }

  ​

      void
      maxTime(double t)

  {
    data_.maxTime(t);
  }

  ​

      const SubscriberData<Data>
      data() const noexcept

  {
    std::lock_guard<std::mutex> l(valueMutex_);

    return data_;
  }

​

    protected :

    void
    value(const Data & data)

  {
    std::lock_guard<std::mutex> l(valueMutex_);

    data_.value(data);
  }

  ​

      void
      value(const Data && data)

  {
    std::lock_guard<std::mutex> l(valueMutex_);

    data_.value(data);
  }

​

    private :

    SubscriberData<Data>
        data_;

  mutable std::mutex valueMutex_;
};

​

    template<typename ROSMessageType, typename TargetType>

    struct ROSSubscriber : public Subscriber<TargetType>

{
  template<typename ConverterFun>

  ROSSubscriber(ConverterFun && fun) : converter_(fun)

  {
  }

  ​

      void
      subscribe(ros::NodeHandle & nh, const std::string & topic, const unsigned bufferSize = 1)

  {
    sub_ = nh.subscribe(topic, bufferSize, &ROSSubscriber::callback, this);
  }

  ​

      std::string
      topic() const

  {
    return sub_.getTopic();
  }

  ​

      const ros::Subscriber &
      subscriber() const

  {
    return sub_;
  }

​

    protected :

    void
    callback(const boost::shared_ptr<ROSMessageType const> & msg)

  {
    this->value(converter_(*msg));
  }

​

    protected :

    ros::Subscriber sub_;

  std::function<TargetType(const ROSMessageType &)> converter_;
};

​

    struct ROSPoseStampedSubscriber : public ROSSubscriber<geometry_msgs::PoseStamped, sva::PTransformd>

{
  ROSPoseStampedSubscriber()

  : ROSSubscriber(
        [](const geometry_msgs::PoseStamped & msg)
        {
          const auto & t = msg.pose.position;

          const auto & r = msg.pose.orientation;

          auto pose =
              sva::PTransformd(Eigen::Quaterniond{r.w, r.x, r.y, r.z}.inverse(), Eigen::Vector3d{t.x, t.y, t.z});

          return pose;
        })

  {
  }
};

​

    struct ROSAccelStampedSubscriber : public ROSSubscriber<geometry_msgs::AccelStamped, sva::MotionVecd>

{
  ROSAccelStampedSubscriber()

  : ROSSubscriber(
        [](const geometry_msgs::AccelStamped & msg)
        {
          const auto & a = msg.accel.linear;

          const auto & w = msg.accel.angular;

          auto acc = sva::MotionVecd(Eigen::Vector3d{w.x, w.y, w.z}, Eigen::Vector3d{a.x, a.y, a.z});

          return acc;
        })

  {
  }
};

​

    struct ROSBoolSubscriber : public ROSSubscriber<std_msgs::Bool, bool>

{
  ROSBoolSubscriber() : ROSSubscriber([](const std_msgs::Bool & msg) { return msg.data; }) {}
};

​

    struct ROSMultiArraySubscriber : public ROSSubscriber<std_msgs::Float32MultiArray, std::vector<float>>

{
  ROSMultiArraySubscriber() : ROSSubscriber([](const std_msgs::Float32MultiArray & msg) { return msg.data; }) {}
};

​

    struct ROSFloatSubscriber : public ROSSubscriber<std_msgs::Float64, double>

{
  ROSFloatSubscriber() : ROSSubscriber([](const std_msgs::Float64 & msg) { return msg.data; }) {}
};

​

    /**

     * @brief Raw data obtained from the Occulus joystick

     */

    struct OcculusHandJoystick

{
  // Actual inputs

  double primary_trigger = 0;

  double secondary_trigger = 0;

  double vertical = 0;

  double horizontal = 0;

  bool x = false;

  bool y = false;

  ​

      // Stateful action

      bool xClicked = false;

  bool yClicked = false;

  bool primary_trigger_clicked = false;

  bool secondary_trigger_clicked = false;

  bool primary_trigger_pressed = false;

  bool primary_trigger_released = false;

  bool secondary_trigger_pressed = false;

  bool secondary_trigger_released = false;

  double primary_trigger_pressed_duration = 0;

  double secondary_trigger_pressed_duration = 0;
};

​

    std::ostream &
    operator<<(std::ostream & os, const OcculusHandJoystick & joy)

{
  os << fmt::format("Primary trigger: {}\nSecondary trigger: {}\nVertical: "
                    "{}\nHorizontal: {}\nX: {} (clicked: {})\nY: "

                    "{} (clicked {})",

                    joy.primary_trigger, joy.secondary_trigger, joy.vertical, joy.horizontal, joy.x, joy.xClicked,

                    joy.y, joy.yClicked);

  return os;
}

​

    /**

     * @brief ROS Subscriber for the left hand occulus joystick

     */

    struct ROSOcculusLeftHandJoySubscriber : public ROSSubscriber<sensor_msgs::Joy, OcculusHandJoystick>

{
  ROSOcculusLeftHandJoySubscriber()

  : ROSSubscriber(
        [this](const sensor_msgs::Joy & msg)
        {
          auto joy = OcculusHandJoystick{};

          // Axes

          joy.primary_trigger = msg.axes[0];

          joy.secondary_trigger = msg.axes[1];

          joy.vertical = msg.axes[2];

          joy.horizontal = msg.axes[3];

          // Buttons

          joy.x = msg.buttons[0];

          joy.y = msg.buttons[1];

          return joy;
        })

  {
  }
};

​

    /**

     * @brief ROS Subscriber for the right hand occulus joystick

     */

    struct ROSOcculusRightHandJoySubscriber : public ROSSubscriber<sensor_msgs::Joy, OcculusHandJoystick>

{
  ROSOcculusRightHandJoySubscriber()

  : ROSSubscriber(
        [this](const sensor_msgs::Joy & msg)
        {
          auto joy = OcculusHandJoystick{};

          joy.primary_trigger = msg.axes[4];

          joy.secondary_trigger = msg.axes[5];

          joy.vertical = msg.axes[6];

          joy.horizontal = msg.axes[7];

          joy.x = msg.buttons[2];

          joy.y = msg.buttons[3];

          return joy;
        })

  {
  }
};

​

    struct OcculusStatefulJoystick

{
  void data(const OcculusHandJoystick & data)

  {
    prevData_ = data_;

    data_ = data;

    if(prevData_.x == false && data_.x == true && !prevData_.xClicked)

    {
      data_.xClicked = true;
    }

    if(prevData_.y == false && data_.y == true && !prevData_.yClicked)

    {
      data_.yClicked = true;
    }

    if(!prevData_.primary_trigger_clicked && prevData_.primary_trigger < trigger_click_threshold

       && data.primary_trigger >= trigger_click_threshold

       && prevData_.primary_trigger_pressed_duration < trigger_max_click_duration)

    {
      data_.primary_trigger_clicked = true;
    }

    if(!prevData_.secondary_trigger_clicked && prevData_.secondary_trigger < trigger_click_threshold

       && data.secondary_trigger >= trigger_click_threshold

       && prevData_.secondary_trigger_pressed_duration < trigger_max_click_duration)

    {
      data_.secondary_trigger_clicked = true;
    }

    data_.primary_trigger_pressed = data_.primary_trigger > trigger_press_threshold;

    data_.secondary_trigger_pressed = data_.secondary_trigger > trigger_press_threshold;

    if(data_.primary_trigger_pressed)

    {
      data_.primary_trigger_pressed_duration = prevData_.primary_trigger_pressed_duration + dt_;
    }

    if(data_.secondary_trigger_pressed)

    {
      data_.secondary_trigger_pressed_duration = prevData_.secondary_trigger_pressed_duration + dt_;
    }

    if(prevData_.primary_trigger_pressed && !data_.primary_trigger_pressed)

    {
      data_.primary_trigger_released = true;
    }

    if(prevData_.secondary_trigger_pressed && !data_.secondary_trigger_pressed)

    {
      data_.secondary_trigger_released = true;
    }
  }

  ​

      const OcculusHandJoystick &
      data() const noexcept

  {
    return data_;
  }

  ​

      void
      dt(double dt)

  {
    dt_ = dt;
  }

  double dt() const noexcept

  {
    return dt_;
  }

​

    protected :

    OcculusHandJoystick prevData_;

  OcculusHandJoystick data_;

  double trigger_click_threshold = 0.98;

  double trigger_press_threshold = 0.01;

  double trigger_max_click_duration = 0.5;

  double dt_ = 0.005;
};

​

    struct PS4Joystick

{
  bool circle = false;

  bool triangle = false;
};

​

    /**

     * @brief ROS Subscriber for the right hand occulus joystick

     */

    struct ROSPS4JoySubscriber : public ROSSubscriber<sensor_msgs::Joy, PS4Joystick>

{
  ROSPS4JoySubscriber()

  : ROSSubscriber(
        [this](const sensor_msgs::Joy & msg)
        {
          auto joy = PS4Joystick{};

          joy.circle = msg.buttons[0];

          joy.triangle = msg.buttons[2];

          return joy;
        })

  {
  }
};
