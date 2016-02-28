# Jaws 2 Messages

*Contains custom-generated messages for Jaws 2.*

Add `jaws_msgs` as dependency like you would any other set of messages.

# Message API/Definitions

*Read them as you would the one for [Vector3](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html).*

## jaws_msgs/Thrust.msg Message

### File: `jaws_msgs/msg/Thrust.msg`

### Raw Message Definition

    float32 aft
    float32 stbd
    float32 port

## jaws_msgs/ThrusStampedt.msg Message

### File: `jaws_msgs/msg/ThrustStamped.msg`

### Raw Message Definition

    Header header
    Thrust thrust

## jaws_msgs/Pwm.msg Message

### File: `jaws_msgs/msg/Pwm.msg`

### Raw Message Definition

    int16 aft
    int16 stbd
    int16 port
    
## jaws_msgs/PwmStamped.msg Message

### File: `jaws_msgs/msg/PwmStamped.msg`

### Raw Message Definition

    Header header
    Pwm pwm
