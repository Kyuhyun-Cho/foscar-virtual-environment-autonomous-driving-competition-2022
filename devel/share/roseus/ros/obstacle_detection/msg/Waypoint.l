;; Auto-generated. Do not edit!


(when (boundp 'obstacle_detection::Waypoint)
  (if (not (find-package "OBSTACLE_DETECTION"))
    (make-package "OBSTACLE_DETECTION"))
  (shadow 'Waypoint (find-package "OBSTACLE_DETECTION")))
(unless (find-package "OBSTACLE_DETECTION::WAYPOINT")
  (make-package "OBSTACLE_DETECTION::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html


(defclass obstacle_detection::Waypoint
  :super ros::object
  :slots (_cnt _x_arr _y_arr ))

(defmethod obstacle_detection::Waypoint
  (:init
   (&key
    ((:cnt __cnt) 0)
    ((:x_arr __x_arr) (make-array 200 :initial-element 0.0 :element-type :float))
    ((:y_arr __y_arr) (make-array 200 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _cnt (round __cnt))
   (setq _x_arr __x_arr)
   (setq _y_arr __y_arr)
   self)
  (:cnt
   (&optional __cnt)
   (if __cnt (setq _cnt __cnt)) _cnt)
  (:x_arr
   (&optional __x_arr)
   (if __x_arr (setq _x_arr __x_arr)) _x_arr)
  (:y_arr
   (&optional __y_arr)
   (if __y_arr (setq _y_arr __y_arr)) _y_arr)
  (:serialization-length
   ()
   (+
    ;; int32 _cnt
    4
    ;; float32[200] _x_arr
    (* 4    200)
    ;; float32[200] _y_arr
    (* 4    200)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _cnt
       (write-long _cnt s)
     ;; float32[200] _x_arr
     (dotimes (i 200)
       (sys::poke (elt _x_arr i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[200] _y_arr
     (dotimes (i 200)
       (sys::poke (elt _y_arr i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _cnt
     (setq _cnt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[200] _x_arr
   (dotimes (i (length _x_arr))
     (setf (elt _x_arr i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[200] _y_arr
   (dotimes (i (length _y_arr))
     (setf (elt _y_arr i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get obstacle_detection::Waypoint :md5sum-) "ef3b913564c2c1d5123a607b0c2cca14")
(setf (get obstacle_detection::Waypoint :datatype-) "obstacle_detection/Waypoint")
(setf (get obstacle_detection::Waypoint :definition-)
      "int32 cnt
float32[200] x_arr
float32[200] y_arr

")



(provide :obstacle_detection/Waypoint "ef3b913564c2c1d5123a607b0c2cca14")


