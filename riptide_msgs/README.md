# Riptide Messages

*Contains custom-generated messages for Riptide.*

# Message API/Definitions

*Read them as you would the one for [Vector3](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html).*

## riptide_msgs/Thrust.msg Message

### File: `riptide_msgs/msg/Thrust.msg`

### Raw Message Definition

    float32 aft
    float32 stbd
    float32 port

## riptide_msgs/ThrustStamped.msg Message

### File: `riptide_msgs/msg/ThrustStamped.msg`

### Raw Message Definition

    Header header
    Thrust thrust

## riptide_msgs/Pwm.msg Message

### File: `riptide_msgs/msg/Pwm.msg`

### Raw Message Definition

    int16 aft
    int16 stbd
    int16 port
    
## riptide_msgs/PwmStamped.msg Message

### File: `riptide_msgs/msg/PwmStamped.msg`

### Raw Message Definition

    Header header
    Pwm pwm
