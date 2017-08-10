#include <unistd.h>
#include "ros/ros.h"
#include <string>
#include "stdio.h"
#include "string.h"
#include "riptide_msgs/ThrustStamped.h"

//Define test thrust
#define TEST_THRUST 1.5

//Constants used for colored text
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_RESET "\x1b[0m"

/**
 * This program is used to validate the function and mapping of the thrusters.
 */
int main(int argc, char **argv)
{
  //create message
  //riptide_msgs::PwmStamped us;
  ros::init(argc, argv, "thruster_validation");

  ros::NodeHandle n;

  ros::Publisher thruster_pub = n.advertise<riptide_msgs::ThrustStamped>("command/thrust", 1000);

    /*publisher used to send to ros, make message, publish
    rosrun package file
    */

riptide_msgs::ThrustStamped thrust;

//Array to move test value and 0 for all other thrusters
float thrust_val[10] = {0};
char names[10][20] = {"surge_port_hi","surge_stbd_hi","surge_port_lo","surge_stbd_lo","sway_fwd","sway_aft","heave_port_fwd","heave_stbd_fwd","heave_port_aft","heave_stbd_aft"};
printf("%s\n", names[0]);

//Warn user that thrusters will run
printf("\n" ANSI_COLOR_YELLOW "WARNING: This node will attempt to run the thrusters. Please be safe." ANSI_COLOR_RESET "\n\n");


int i;
char response [10];
//loop for all thrusters
for(i=0; i<10; i++){
    //prompt user
    printf("Thruster " ANSI_COLOR_YELLOW "%s " ANSI_COLOR_RESET "will run. OK? y/n/exit\n",names[i]);
    scanf("%s", response);
    
    while((strcmp(response,"y") != 0) && (strcmp(response,"n")!=0) && (strcmp(response, "exit"))){
        printf("Invalid input. y/n/exit, yes (run thruster), no (skip thruster), exit (kill all thrusters and exit)\n");
        scanf("%s", response);
    }     

    //Check response
    if ((strcmp(response,"y") == 0) || (strcmp(response,"\n")==0)){
        printf("RUNNING\n");
        //Run thruster for 2 seconds
        //Start
        thrust_val[i] = TEST_THRUST;
        //thrust.force.THRUSTER_NAME = 0 or TEST_THRUST
        thrust.force.surge_port_hi = thrust_val[0];
        thrust.force.surge_stbd_hi = thrust_val[1];
        thrust.force.surge_port_lo = thrust_val[2];
        thrust.force.surge_stbd_lo = thrust_val[3];
        thrust.force.sway_fwd = thrust_val[4];
        thrust.force.sway_aft = thrust_val[5];
        thrust.force.heave_port_fwd = thrust_val[6];
        thrust.force.heave_stbd_fwd = thrust_val[7];
        thrust.force.heave_port_aft = thrust_val[8];
        thrust.force.heave_stbd_aft = thrust_val[9];

        //publish message
        thruster_pub.publish(thrust); 
        
        //Wait
        sleep(1);

        //Turn off
        thrust_val[i] = 0;
        //thrust.force.THRUSTER_NAME = 0 or TEST_THRUST
        thrust.force.surge_port_hi = thrust_val[0];
        thrust.force.surge_stbd_hi = thrust_val[1];
        thrust.force.surge_port_lo = thrust_val[2];
        thrust.force.surge_stbd_lo = thrust_val[3];
        thrust.force.sway_fwd = thrust_val[4];
        thrust.force.sway_aft = thrust_val[5];
        thrust.force.heave_port_fwd = thrust_val[6];
        thrust.force.heave_stbd_fwd = thrust_val[7];
        thrust.force.heave_port_aft = thrust_val[8];
        thrust.force.heave_stbd_aft = thrust_val[9];

        //publish message
        thruster_pub.publish(thrust);         
        printf("THRUSTER STOPPED\n\n");

    }else if (strcmp(response,"n") == 0){
        printf("SKIPPING\n\n");

    }else if (strcmp(response,"exit") == 0){
        printf("EXITING\n");
        //Stop all thrusters
        //thrust.force.THRUSTER_NAME = 0 or TEST_THRUST
        thrust.force.surge_port_hi = 0;
        thrust.force.surge_stbd_hi = 0;
        thrust.force.surge_port_lo = 0;
        thrust.force.surge_stbd_lo = 0;
        thrust.force.sway_fwd = 0;
        thrust.force.sway_aft = 0;
        thrust.force.heave_port_fwd = 0;
        thrust.force.heave_stbd_fwd = 0;
        thrust.force.heave_port_aft = 0;
        thrust.force.heave_stbd_aft = 0;

        //publish message
        thruster_pub.publish(thrust); 
       
        return 0;
    }
} //End For loop

//Stop all thrusters
//thrust.force.THRUSTER_NAME = 0 or TEST_THRUST
thrust.force.surge_port_hi = 0;
thrust.force.surge_stbd_hi = 0;
thrust.force.surge_port_lo = 0;
thrust.force.surge_stbd_lo = 0;
thrust.force.sway_fwd = 0;
thrust.force.sway_aft = 0;
thrust.force.heave_port_fwd = 0;
thrust.force.heave_stbd_fwd = 0;
thrust.force.heave_port_aft = 0;
thrust.force.heave_stbd_aft = 0;

//publish message
thruster_pub.publish(thrust); 

printf("All thrusters have run\n");
//end
  return 0;
}


//From different file, reference of how thrusters are run
/*us.pwm.surge_port_hi = clockwise(thrust->force.surge_port_hi);
us.pwm.surge_stbd_hi = counterclockwise(thrust->force.surge_stbd_hi);
us.pwm.surge_port_lo = counterclockwise(thrust->force.surge_port_lo);
us.pwm.surge_stbd_lo = clockwise(thrust->force.surge_stbd_lo);
us.pwm.sway_fwd = counterclockwise(thrust->force.sway_fwd);
us.pwm.sway_aft = clockwise(thrust->force.sway_aft);
us.pwm.heave_port_fwd = counterclockwise(thrust->force.heave_port_fwd);
us.pwm.heave_stbd_fwd = clockwise(thrust->force.heave_stbd_fwd);
us.pwm.heave_port_aft = clockwise(thrust->force.heave_port_aft);
us.pwm.heave_stbd_aft = counterclockwise(thrust->force.heave_stbd_aft);
pwm.publish(us);
*/
