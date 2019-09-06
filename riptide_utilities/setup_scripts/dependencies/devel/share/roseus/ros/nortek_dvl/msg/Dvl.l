;; Auto-generated. Do not edit!


(when (boundp 'nortek_dvl::Dvl)
  (if (not (find-package "NORTEK_DVL"))
    (make-package "NORTEK_DVL"))
  (shadow 'Dvl (find-package "NORTEK_DVL")))
(unless (find-package "NORTEK_DVL::DVL")
  (make-package "NORTEK_DVL::DVL"))

(in-package "ROS")
;;//! \htmlinclude Dvl.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass nortek_dvl::Dvl
  :super ros::object
  :slots (_header _time _dt1 _dt2 _velocity _figureOfMerit _beamDistance _batteryVoltage _speedSound _pressure _temp ))

(defmethod nortek_dvl::Dvl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) 0.0)
    ((:dt1 __dt1) 0.0)
    ((:dt2 __dt2) 0.0)
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:figureOfMerit __figureOfMerit) 0.0)
    ((:beamDistance __beamDistance) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:batteryVoltage __batteryVoltage) 0.0)
    ((:speedSound __speedSound) 0.0)
    ((:pressure __pressure) 0.0)
    ((:temp __temp) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time (float __time))
   (setq _dt1 (float __dt1))
   (setq _dt2 (float __dt2))
   (setq _velocity __velocity)
   (setq _figureOfMerit (float __figureOfMerit))
   (setq _beamDistance __beamDistance)
   (setq _batteryVoltage (float __batteryVoltage))
   (setq _speedSound (float __speedSound))
   (setq _pressure (float __pressure))
   (setq _temp (float __temp))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:dt1
   (&optional __dt1)
   (if __dt1 (setq _dt1 __dt1)) _dt1)
  (:dt2
   (&optional __dt2)
   (if __dt2 (setq _dt2 __dt2)) _dt2)
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:figureOfMerit
   (&optional __figureOfMerit)
   (if __figureOfMerit (setq _figureOfMerit __figureOfMerit)) _figureOfMerit)
  (:beamDistance
   (&optional __beamDistance)
   (if __beamDistance (setq _beamDistance __beamDistance)) _beamDistance)
  (:batteryVoltage
   (&optional __batteryVoltage)
   (if __batteryVoltage (setq _batteryVoltage __batteryVoltage)) _batteryVoltage)
  (:speedSound
   (&optional __speedSound)
   (if __speedSound (setq _speedSound __speedSound)) _speedSound)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _time
    8
    ;; float64 _dt1
    8
    ;; float64 _dt2
    8
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; float64 _figureOfMerit
    8
    ;; float64[4] _beamDistance
    (* 8    4)
    ;; float64 _batteryVoltage
    8
    ;; float64 _speedSound
    8
    ;; float64 _pressure
    8
    ;; float64 _temp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dt1
       (sys::poke _dt1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dt2
       (sys::poke _dt2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; float64 _figureOfMerit
       (sys::poke _figureOfMerit (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[4] _beamDistance
     (dotimes (i 4)
       (sys::poke (elt _beamDistance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _batteryVoltage
       (sys::poke _batteryVoltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speedSound
       (sys::poke _speedSound (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pressure
       (sys::poke _pressure (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temp
       (sys::poke _temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dt1
     (setq _dt1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dt2
     (setq _dt2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float64 _figureOfMerit
     (setq _figureOfMerit (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[4] _beamDistance
   (dotimes (i (length _beamDistance))
     (setf (elt _beamDistance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _batteryVoltage
     (setq _batteryVoltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speedSound
     (setq _speedSound (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pressure
     (setq _pressure (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temp
     (setq _temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get nortek_dvl::Dvl :md5sum-) "b1ec29ea990029dc78359fd83b508011")
(setf (get nortek_dvl::Dvl :datatype-) "nortek_dvl/Dvl")
(setf (get nortek_dvl::Dvl :definition-)
      "Header header
float64 time    #ping time
float64 dt1     #time from trigger to center of water track cell
float64 dt2     #time from start of output message to center of water track cell
geometry_msgs/Vector3 velocity
float64 figureOfMerit
float64[4] beamDistance
float64 batteryVoltage
float64 speedSound
float64 pressure
float64 temp
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :nortek_dvl/Dvl "b1ec29ea990029dc78359fd83b508011")


