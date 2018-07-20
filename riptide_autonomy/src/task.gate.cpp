#include "riptide_autonomy/task.gate.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tslam");
  Gate g;
  ros::spin();
}

Gate::Gate() : nh("tslam") { // NOTE: there is no namespace declared in nh()
	go_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &Gate::Go, this);
	//abort_sub = nh.subscribe<std_msgs::Empty>("/command/tslam/abort", 1, &Gate::Abort, this);
}

void Gate::Go(const riptide_msgs::TaskInfo::ConstPtr& task)
{
	if(task->task_id == 0)
	{
		ros::Publisher tslam_pub = nh.advertise<std_msgs::Int8>("/command/tslam/go", 1);
		std_msgs::Int8 msg;
		msg.data = 0;
		tslam_pub.publish(msg);
		tslam_pub.shutdown();

		yolo_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Gate::YoloCB, this);
	}
}

void Gate::YoloCB(darknet_ros_msgs::BoundingBoxes bboxes)
{
	// Once we are at heading
	if (bboxes.bounding_boxes.size() != 0)
	{
		yolo_sub.shutdown();

		ros::Publisher tslam_pub = nh.advertise<std_msgs::Empty>("/command/tslam/abort", 1);
		std_msgs::Empty msg;
		tslam_pub.publish(msg);
		tslam_pub.shutdown();

		
	}
}

