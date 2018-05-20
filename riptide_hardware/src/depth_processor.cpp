#include "riptide_hardware/depth_processor.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "depth_processor");
 DepthProcessor dp;
 dp.loop();
}

//Constructor
DepthProcessor::DepthProcessor() : nh()
{
 depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth/raw", 1, &DepthProcessor::DepthCB, this);
 state_depth_pub = nh.advertise<riptide_msgs::Depth>("state/depth", 1);
 c = 2; //center of smoothed data
 cycles = 1;
 size = 5;
}

//Callback
void DepthProcessor::DepthCB(const riptide_msgs::Depth::ConstPtr& depth)
{
  /*
    Load values into the state, it holds:
    std_msgs/Header header
    float32 depth
    float32 pressure
    float32 temp
    float32 altitude
  */
  state[0].header = depth->header;
  state[0].depth = depth->depth;
  state[0].pressure = depth->pressure;
  state[0].temp = depth->temp;
  state[0].altitude = depth->altitude;

  ds[0] = state[0].depth;
  //gaussian smooth of 5 points, needs five points minumum
  if (cycles >= size){
    smoothData();
  }
  //add cycles till we get 10 smooth points
  if (cycles <10) {
    cycles += 1;
  }

  //publish smoothed data to state/depth
  state_depth_pub.publish(state[c]);
  //shift values in the array to make room, freshest value is always spot 0
  for (int i=4; i >0; i--){
    state[i] = state[i-1];
    ds[i] = ds[i-1];
  }

}

void DepthProcessor::smoothData() {
  int coef[size] = {1, 3, 6, 3, 1};
  int sumCoef = 14;
  //smooth this depth
  state[c].depth = 0;
  for (int i = 0; i < size; i++) {
    state[c].depth += coef[i]*ds[i]/sumCoef;
  }
}

void DepthProcessor::loop(){
  ros::Rate rate(1000);
  while (!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }
}
