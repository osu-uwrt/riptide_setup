;; Auto-generated. Do not edit!


(when (boundp 'nortek_dvl::DvlStatus)
  (if (not (find-package "NORTEK_DVL"))
    (make-package "NORTEK_DVL"))
  (shadow 'DvlStatus (find-package "NORTEK_DVL")))
(unless (find-package "NORTEK_DVL::DVLSTATUS")
  (make-package "NORTEK_DVL::DVLSTATUS"))

(in-package "ROS")
;;//! \htmlinclude DvlStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass nortek_dvl::DvlStatus
  :super ros::object
  :slots (_header _b1_vel_valid _b2_vel_valid _b3_vel_valid _b4_vel_valid _b1_dist_valid _b2_dist_valid _b3_dist_valid _b4_dist_valid _b1_fom_valid _b2_fom_valid _b3_fom_valid _b4_fom_valid _x_vel_valid _y_vel_valid _z1_vel_valid _z2_vel_valid _x_fom_valid _y_fom_valid _z1_fom_valid _z2_fom_valid _proc_cap _wakeup_state ))

(defmethod nortek_dvl::DvlStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:b1_vel_valid __b1_vel_valid) nil)
    ((:b2_vel_valid __b2_vel_valid) nil)
    ((:b3_vel_valid __b3_vel_valid) nil)
    ((:b4_vel_valid __b4_vel_valid) nil)
    ((:b1_dist_valid __b1_dist_valid) nil)
    ((:b2_dist_valid __b2_dist_valid) nil)
    ((:b3_dist_valid __b3_dist_valid) nil)
    ((:b4_dist_valid __b4_dist_valid) nil)
    ((:b1_fom_valid __b1_fom_valid) nil)
    ((:b2_fom_valid __b2_fom_valid) nil)
    ((:b3_fom_valid __b3_fom_valid) nil)
    ((:b4_fom_valid __b4_fom_valid) nil)
    ((:x_vel_valid __x_vel_valid) nil)
    ((:y_vel_valid __y_vel_valid) nil)
    ((:z1_vel_valid __z1_vel_valid) nil)
    ((:z2_vel_valid __z2_vel_valid) nil)
    ((:x_fom_valid __x_fom_valid) nil)
    ((:y_fom_valid __y_fom_valid) nil)
    ((:z1_fom_valid __z1_fom_valid) nil)
    ((:z2_fom_valid __z2_fom_valid) nil)
    ((:proc_cap __proc_cap) 0)
    ((:wakeup_state __wakeup_state) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _b1_vel_valid __b1_vel_valid)
   (setq _b2_vel_valid __b2_vel_valid)
   (setq _b3_vel_valid __b3_vel_valid)
   (setq _b4_vel_valid __b4_vel_valid)
   (setq _b1_dist_valid __b1_dist_valid)
   (setq _b2_dist_valid __b2_dist_valid)
   (setq _b3_dist_valid __b3_dist_valid)
   (setq _b4_dist_valid __b4_dist_valid)
   (setq _b1_fom_valid __b1_fom_valid)
   (setq _b2_fom_valid __b2_fom_valid)
   (setq _b3_fom_valid __b3_fom_valid)
   (setq _b4_fom_valid __b4_fom_valid)
   (setq _x_vel_valid __x_vel_valid)
   (setq _y_vel_valid __y_vel_valid)
   (setq _z1_vel_valid __z1_vel_valid)
   (setq _z2_vel_valid __z2_vel_valid)
   (setq _x_fom_valid __x_fom_valid)
   (setq _y_fom_valid __y_fom_valid)
   (setq _z1_fom_valid __z1_fom_valid)
   (setq _z2_fom_valid __z2_fom_valid)
   (setq _proc_cap (round __proc_cap))
   (setq _wakeup_state (string __wakeup_state))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:b1_vel_valid
   (&optional __b1_vel_valid)
   (if __b1_vel_valid (setq _b1_vel_valid __b1_vel_valid)) _b1_vel_valid)
  (:b2_vel_valid
   (&optional __b2_vel_valid)
   (if __b2_vel_valid (setq _b2_vel_valid __b2_vel_valid)) _b2_vel_valid)
  (:b3_vel_valid
   (&optional __b3_vel_valid)
   (if __b3_vel_valid (setq _b3_vel_valid __b3_vel_valid)) _b3_vel_valid)
  (:b4_vel_valid
   (&optional __b4_vel_valid)
   (if __b4_vel_valid (setq _b4_vel_valid __b4_vel_valid)) _b4_vel_valid)
  (:b1_dist_valid
   (&optional __b1_dist_valid)
   (if __b1_dist_valid (setq _b1_dist_valid __b1_dist_valid)) _b1_dist_valid)
  (:b2_dist_valid
   (&optional __b2_dist_valid)
   (if __b2_dist_valid (setq _b2_dist_valid __b2_dist_valid)) _b2_dist_valid)
  (:b3_dist_valid
   (&optional __b3_dist_valid)
   (if __b3_dist_valid (setq _b3_dist_valid __b3_dist_valid)) _b3_dist_valid)
  (:b4_dist_valid
   (&optional __b4_dist_valid)
   (if __b4_dist_valid (setq _b4_dist_valid __b4_dist_valid)) _b4_dist_valid)
  (:b1_fom_valid
   (&optional __b1_fom_valid)
   (if __b1_fom_valid (setq _b1_fom_valid __b1_fom_valid)) _b1_fom_valid)
  (:b2_fom_valid
   (&optional __b2_fom_valid)
   (if __b2_fom_valid (setq _b2_fom_valid __b2_fom_valid)) _b2_fom_valid)
  (:b3_fom_valid
   (&optional __b3_fom_valid)
   (if __b3_fom_valid (setq _b3_fom_valid __b3_fom_valid)) _b3_fom_valid)
  (:b4_fom_valid
   (&optional __b4_fom_valid)
   (if __b4_fom_valid (setq _b4_fom_valid __b4_fom_valid)) _b4_fom_valid)
  (:x_vel_valid
   (&optional __x_vel_valid)
   (if __x_vel_valid (setq _x_vel_valid __x_vel_valid)) _x_vel_valid)
  (:y_vel_valid
   (&optional __y_vel_valid)
   (if __y_vel_valid (setq _y_vel_valid __y_vel_valid)) _y_vel_valid)
  (:z1_vel_valid
   (&optional __z1_vel_valid)
   (if __z1_vel_valid (setq _z1_vel_valid __z1_vel_valid)) _z1_vel_valid)
  (:z2_vel_valid
   (&optional __z2_vel_valid)
   (if __z2_vel_valid (setq _z2_vel_valid __z2_vel_valid)) _z2_vel_valid)
  (:x_fom_valid
   (&optional __x_fom_valid)
   (if __x_fom_valid (setq _x_fom_valid __x_fom_valid)) _x_fom_valid)
  (:y_fom_valid
   (&optional __y_fom_valid)
   (if __y_fom_valid (setq _y_fom_valid __y_fom_valid)) _y_fom_valid)
  (:z1_fom_valid
   (&optional __z1_fom_valid)
   (if __z1_fom_valid (setq _z1_fom_valid __z1_fom_valid)) _z1_fom_valid)
  (:z2_fom_valid
   (&optional __z2_fom_valid)
   (if __z2_fom_valid (setq _z2_fom_valid __z2_fom_valid)) _z2_fom_valid)
  (:proc_cap
   (&optional __proc_cap)
   (if __proc_cap (setq _proc_cap __proc_cap)) _proc_cap)
  (:wakeup_state
   (&optional __wakeup_state)
   (if __wakeup_state (setq _wakeup_state __wakeup_state)) _wakeup_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _b1_vel_valid
    1
    ;; bool _b2_vel_valid
    1
    ;; bool _b3_vel_valid
    1
    ;; bool _b4_vel_valid
    1
    ;; bool _b1_dist_valid
    1
    ;; bool _b2_dist_valid
    1
    ;; bool _b3_dist_valid
    1
    ;; bool _b4_dist_valid
    1
    ;; bool _b1_fom_valid
    1
    ;; bool _b2_fom_valid
    1
    ;; bool _b3_fom_valid
    1
    ;; bool _b4_fom_valid
    1
    ;; bool _x_vel_valid
    1
    ;; bool _y_vel_valid
    1
    ;; bool _z1_vel_valid
    1
    ;; bool _z2_vel_valid
    1
    ;; bool _x_fom_valid
    1
    ;; bool _y_fom_valid
    1
    ;; bool _z1_fom_valid
    1
    ;; bool _z2_fom_valid
    1
    ;; int8 _proc_cap
    1
    ;; string _wakeup_state
    4 (length _wakeup_state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _b1_vel_valid
       (if _b1_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b2_vel_valid
       (if _b2_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b3_vel_valid
       (if _b3_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b4_vel_valid
       (if _b4_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b1_dist_valid
       (if _b1_dist_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b2_dist_valid
       (if _b2_dist_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b3_dist_valid
       (if _b3_dist_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b4_dist_valid
       (if _b4_dist_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b1_fom_valid
       (if _b1_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b2_fom_valid
       (if _b2_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b3_fom_valid
       (if _b3_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _b4_fom_valid
       (if _b4_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _x_vel_valid
       (if _x_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _y_vel_valid
       (if _y_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _z1_vel_valid
       (if _z1_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _z2_vel_valid
       (if _z2_vel_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _x_fom_valid
       (if _x_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _y_fom_valid
       (if _y_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _z1_fom_valid
       (if _z1_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _z2_fom_valid
       (if _z2_fom_valid (write-byte -1 s) (write-byte 0 s))
     ;; int8 _proc_cap
       (write-byte _proc_cap s)
     ;; string _wakeup_state
       (write-long (length _wakeup_state) s) (princ _wakeup_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _b1_vel_valid
     (setq _b1_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b2_vel_valid
     (setq _b2_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b3_vel_valid
     (setq _b3_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b4_vel_valid
     (setq _b4_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b1_dist_valid
     (setq _b1_dist_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b2_dist_valid
     (setq _b2_dist_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b3_dist_valid
     (setq _b3_dist_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b4_dist_valid
     (setq _b4_dist_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b1_fom_valid
     (setq _b1_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b2_fom_valid
     (setq _b2_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b3_fom_valid
     (setq _b3_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _b4_fom_valid
     (setq _b4_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _x_vel_valid
     (setq _x_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _y_vel_valid
     (setq _y_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _z1_vel_valid
     (setq _z1_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _z2_vel_valid
     (setq _z2_vel_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _x_fom_valid
     (setq _x_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _y_fom_valid
     (setq _y_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _z1_fom_valid
     (setq _z1_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _z2_fom_valid
     (setq _z2_fom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _proc_cap
     (setq _proc_cap (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _proc_cap 127) (setq _proc_cap (- _proc_cap 256)))
   ;; string _wakeup_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _wakeup_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get nortek_dvl::DvlStatus :md5sum-) "e1789433d0fd4a0c672172f8d32e464d")
(setf (get nortek_dvl::DvlStatus :datatype-) "nortek_dvl/DvlStatus")
(setf (get nortek_dvl::DvlStatus :definition-)
      "Header header

bool b1_vel_valid
bool b2_vel_valid
bool b3_vel_valid
bool b4_vel_valid

bool b1_dist_valid
bool b2_dist_valid
bool b3_dist_valid
bool b4_dist_valid

bool b1_fom_valid
bool b2_fom_valid
bool b3_fom_valid
bool b4_fom_valid

bool x_vel_valid
bool y_vel_valid
bool z1_vel_valid
bool z2_vel_valid

bool x_fom_valid
bool y_fom_valid
bool z1_fom_valid
bool z2_fom_valid

int8 proc_cap
string wakeup_state

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

")



(provide :nortek_dvl/DvlStatus "e1789433d0fd4a0c672172f8d32e464d")


