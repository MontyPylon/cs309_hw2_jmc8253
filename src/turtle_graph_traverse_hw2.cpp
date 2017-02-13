#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cs309_turtlesim/turtle_constants.h>
#include <vector>
#include <cmath>
using namespace std;

//global publisher for velocity
ros::Publisher velocity_pub;

//the turtle's current pose gets stored here
turtlesim::Pose current_pose;
vector<double> remaining_x;
vector<double> remaining_y;
vector<double> next_target;
bool at_target = true;
bool first_msg = false;
bool completedAngle = false;

double linear_vel = 0.1;
double angular_vel = 0.1;

// Stolen from jivko
double euc_distance(double x1, double y1, double x2, double y2){
    return sqrt( (x1- x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );    
}

bool travel_to_target(ros::Publisher velocity_pub, turtlesim::Pose pos, vector<double> next, bool angle) {
    geometry_msgs::Twist msg;

    double d = euc_distance(pos.x, pos.y, next.at(0), next.at(1));
    cout << "x: " << pos.x << ", y: " << pos.y << ", next_x " << next.at(0) << ", next_y: " << next.at(1) << endl;
    cout << "Distance to next: " << d << endl;
    if(d < DISTANCE_THRESHOLD) {
        // We are within the threshold
        cout << "Within threshold, moving to next turtle" << endl;
        msg.linear.x = 0.0;
        return true;
    } else {
        double ratio = (pos.y - next.at(1)) / (next.at(0) - pos.x);
        double thetaCalc = atan(ratio);
        if(thetaCalc < 0) {
            thetaCalc += 2*PI;
        }
        double currentAngle = pos.theta;
        if(pos.theta < 0) {
            currentAngle += 2*PI;
        }

        cout << "Angle between turtle and next target: " << thetaCalc << endl;
        cout << "Current angle: " << currentAngle << endl;

        if(!angle) {
            if(thetaCalc < (currentAngle + 0.1) && thetaCalc > (currentAngle - 0.1)) {
                completedAngle = true;
                cout << "currentAngle: " << currentAngle << ", thetaCalc: " << thetaCalc << endl;
                cout << "FOUND ANGLE" << endl;
                msg.angular.z = 0;
            } else {

                // Decide to go right or left
                double testRight = currentAngle - thetaCalc;
                if(testRight < 0) {
                    testRight += 2*PI;
                }
                double testLeft = thetaCalc - currentAngle;
                if(testLeft < 0) {
                    testLeft += 2*PI;
                }

                //cout << "Test left: " << testLeft << endl;
                //cout << "Test right: " << testRight << endl;

                if(testRight <= testLeft) {
                    msg.angular.z = -angular_vel;
                } else {
                    msg.angular.z = angular_vel;
                }
            }
        } else {
            msg.linear.x = linear_vel;

            // Decide to go right or left
            double testRight = currentAngle - thetaCalc;
            if(testRight < 0) {
                testRight += 2*PI;
            }
            double testLeft = thetaCalc - currentAngle;
            if(testLeft < 0) {
                testLeft += 2*PI;
            }

            //cout << "Test left: " << testLeft << endl;
            //cout << "Test right: " << testRight << endl;

            if(testRight <= testLeft) {
                msg.angular.z = -angular_vel * testRight;
                //cout << "Turn right a little at speed: " << (angular_vel * testRight) << endl;
            } else {
                msg.angular.z = angular_vel * testLeft;
                //cout << "Turn left a little at speed: " << (angular_vel * testLeft) << endl;
            }
        }
        velocity_pub.publish(msg);
        return false;
    }
}

vector<double> find_nearest_neighbor(turtlesim::Pose pos, vector<double> rem_x, vector<double> rem_y) {
    vector<double> returner;

    double shortest = 1000;
    double shortest_x = 1000;
    double shortest_y = 1000;
    double list_num = -1;
    for(int a = 0; a < rem_x.size(); a++) {
        double dist_x = pos.x - rem_x.at(a);
        double dist_y = pos.y - rem_y.at(a);
        double pyth = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
        if(pyth < shortest) {
            shortest = pyth;
            shortest_x = rem_x.at(a);
            shortest_y = rem_y.at(a);
            list_num = a;
        }
    }
    returner.push_back(shortest_x);
    returner.push_back(shortest_y);
    returner.push_back(list_num);
    return returner;   
}

void pose_cb(const turtlesim::Pose::ConstPtr& msg){
    current_pose = *msg;
    first_msg = true;
}

int main(int argc, char **argv){
    
    //initialize the node
    ros::init(argc, argv, "traveling_salesturtle");
    //instantiate the node handle which is used for creating publishers and subscribers
    ros::NodeHandle n;
    
    //publisher for velocity commands
    velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    //subscriber for the turtle's pose
    ros::Subscriber sub_pose = n.subscribe("/turtle1/pose", 1, pose_cb );
    ros::Rate r(10);

    for(int a = 0; a < (sizeof(x_coords)/sizeof(*x_coords)); a++) {
        remaining_x.push_back(x_coords[a]);
        remaining_y.push_back(y_coords[a]);
    }


    while(ros::ok()) {
		
        //cout << "Current x: " << current_pose.x << endl;
        //cout << "Current y: " << current_pose.y << endl;

        // Wait until we have the first message of the turtle's position
        if(first_msg) {
            //cout << current_pose.x << endl;
            if(at_target) {
                at_target = false;

                // Next x and y position
                next_target = find_nearest_neighbor(current_pose, remaining_x, remaining_y);

                cout << "Next x pos: " << next_target.at(0) << endl;
                cout << "Next y pos: " << next_target.at(1) << endl;
                cout << "List num: " << next_target.at(2) << endl;

                // Remove the target x and y from remaining set
                remaining_x.erase(remaining_x.begin() + next_target.at(2));
                remaining_y.erase(remaining_y.begin() + next_target.at(2));

                // Begin to travel to target position
                completedAngle = false;
                at_target = travel_to_target(velocity_pub, current_pose, next_target, completedAngle);

            } else {
                // at_target will be true when the turtle is at the position
                at_target = travel_to_target(velocity_pub, current_pose, next_target, completedAngle);
            }
        }

		r.sleep();
		ros::spinOnce();
	}
    
    return 0;
}
