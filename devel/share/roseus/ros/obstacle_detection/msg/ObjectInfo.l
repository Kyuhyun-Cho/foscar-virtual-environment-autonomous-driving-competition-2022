;; Auto-generated. Do not edit!


(when (boundp 'obstacle_detection::ObjectInfo)
  (if (not (find-package "OBSTACLE_DETECTION"))
    (make-package "OBSTACLE_DETECTION"))
  (shadow 'ObjectInfo (find-package "OBSTACLE_DETECTION")))
(unless (find-package "OBSTACLE_DETECTION::OBJECTINFO")
  (make-package "OBSTACLE_DETECTION::OBJECTINFO"))

(in-package "ROS")
;;//! \htmlinclude ObjectInfo.msg.html


(defclass obstacle_detection::ObjectInfo
  :super ros::object
  :slots (_objectCounts _centerX _centerY _centerZ _lengthX _lengthY _lengthZ ))

(defmethod obstacle_detection::ObjectInfo
  (:init
   (&key
    ((:objectCounts __objectCounts) 0)
    ((:centerX __centerX) (make-array 100 :initial-element 0.0 :element-type :float))
    ((:centerY __centerY) (make-array 100 :initial-element 0.0 :element-type :float))
    ((:centerZ __centerZ) (make-array 100 :initial-element 0.0 :element-type :float))
    ((:lengthX __lengthX) (make-array 100 :initial-element 0.0 :element-type :float))
    ((:lengthY __lengthY) (make-array 100 :initial-element 0.0 :element-type :float))
    ((:lengthZ __lengthZ) (make-array 100 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _objectCounts (round __objectCounts))
   (setq _centerX __centerX)
   (setq _centerY __centerY)
   (setq _centerZ __centerZ)
   (setq _lengthX __lengthX)
   (setq _lengthY __lengthY)
   (setq _lengthZ __lengthZ)
   self)
  (:objectCounts
   (&optional __objectCounts)
   (if __objectCounts (setq _objectCounts __objectCounts)) _objectCounts)
  (:centerX
   (&optional __centerX)
   (if __centerX (setq _centerX __centerX)) _centerX)
  (:centerY
   (&optional __centerY)
   (if __centerY (setq _centerY __centerY)) _centerY)
  (:centerZ
   (&optional __centerZ)
   (if __centerZ (setq _centerZ __centerZ)) _centerZ)
  (:lengthX
   (&optional __lengthX)
   (if __lengthX (setq _lengthX __lengthX)) _lengthX)
  (:lengthY
   (&optional __lengthY)
   (if __lengthY (setq _lengthY __lengthY)) _lengthY)
  (:lengthZ
   (&optional __lengthZ)
   (if __lengthZ (setq _lengthZ __lengthZ)) _lengthZ)
  (:serialization-length
   ()
   (+
    ;; int32 _objectCounts
    4
    ;; float64[100] _centerX
    (* 8    100)
    ;; float64[100] _centerY
    (* 8    100)
    ;; float64[100] _centerZ
    (* 8    100)
    ;; float64[100] _lengthX
    (* 8    100)
    ;; float64[100] _lengthY
    (* 8    100)
    ;; float64[100] _lengthZ
    (* 8    100)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _objectCounts
       (write-long _objectCounts s)
     ;; float64[100] _centerX
     (dotimes (i 100)
       (sys::poke (elt _centerX i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[100] _centerY
     (dotimes (i 100)
       (sys::poke (elt _centerY i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[100] _centerZ
     (dotimes (i 100)
       (sys::poke (elt _centerZ i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[100] _lengthX
     (dotimes (i 100)
       (sys::poke (elt _lengthX i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[100] _lengthY
     (dotimes (i 100)
       (sys::poke (elt _lengthY i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[100] _lengthZ
     (dotimes (i 100)
       (sys::poke (elt _lengthZ i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _objectCounts
     (setq _objectCounts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[100] _centerX
   (dotimes (i (length _centerX))
     (setf (elt _centerX i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[100] _centerY
   (dotimes (i (length _centerY))
     (setf (elt _centerY i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[100] _centerZ
   (dotimes (i (length _centerZ))
     (setf (elt _centerZ i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[100] _lengthX
   (dotimes (i (length _lengthX))
     (setf (elt _lengthX i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[100] _lengthY
   (dotimes (i (length _lengthY))
     (setf (elt _lengthY i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[100] _lengthZ
   (dotimes (i (length _lengthZ))
     (setf (elt _lengthZ i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get obstacle_detection::ObjectInfo :md5sum-) "fcb703df87d24291d755127aee75bb7e")
(setf (get obstacle_detection::ObjectInfo :datatype-) "obstacle_detection/ObjectInfo")
(setf (get obstacle_detection::ObjectInfo :definition-)
      "int32 objectCounts
float64[100] centerX
float64[100] centerY
float64[100] centerZ
float64[100] lengthX
float64[100] lengthY
float64[100] lengthZ

")



(provide :obstacle_detection/ObjectInfo "fcb703df87d24291d755127aee75bb7e")


