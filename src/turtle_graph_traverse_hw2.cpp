#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cs309_turtlesim/turtle_constants.h>
#include <turtlesim/Kill.h>
#include <algorithm>
#include <vector>
#include <cmath>

using namespace std;

//global publisher for velocity
ros::Publisher velocity_pub;

//the turtle's current pose gets stored here
turtlesim::Pose current_pose;
vector<double> next_target;
bool at_target = true;
bool first_msg = false;
double turningConstant = PI / 4;

double linear_vel = MAX_LINEAR_VELOCITY;
double angular_vel = MAX_ANGULAR_VELOCITY;

// Stolen from jivko
double euc_distance(double x1, double y1, double x2, double y2){
    return sqrt( (x1- x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );    
}

bool travel_to_target(ros::Publisher velocity_pub, turtlesim::Pose pos, vector<double> next) {
    geometry_msgs::Twist msg;

    double d = euc_distance(pos.x, pos.y, next.at(0), next.at(1));
    //cout << "x: " << pos.x << ", y: " << pos.y << ", next_x " << next.at(0) << ", next_y: " << next.at(1) << endl;
    //cout << "Distance to next: " << d << endl;
    if(d < DISTANCE_THRESHOLD) {
        // We are within the threshold
        cout << "Within threshold, moving to next turtle" << endl;
        //msg.linear.x = 0.0;
        //msg.angular.z = 0.0;
        //velocity_pub.publish(msg);
        return true;
    } else {
        double ratio = (next.at(1) - pos.y) / (next.at(0) - pos.x);
        //cout << "CurY: " << pos.y << ", TarY: " << next.at(1) << ", CurX: " << pos.x << ", TarX: " << next.at(0) << endl;
        double targetTheta = atan(ratio);
        if((next.at(0) - pos.x) < 0) {
            targetTheta += PI;
        }
        if(targetTheta < 0) {
            targetTheta += 2*PI;
        }
        double currentAngle = pos.theta;
        if(pos.theta < 0) {
            currentAngle += 2*PI;
        }
        //cout << "currentAngle: " << currentAngle << ", targetTheta: " << targetTheta << endl;
        
        double deltaAngle = atan2(sin(targetTheta - currentAngle), cos(targetTheta - currentAngle));
        
        if (deltaAngle < 0) {
            msg.angular.z = -angular_vel;
        } else {
            msg.angular.z = angular_vel;
        }

        if(abs(deltaAngle) < turningConstant) {
            msg.linear.x = linear_vel;
        }

        velocity_pub.publish(msg);
        return false;
    }
}

double find_distance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    return dist;
}

double find_total_distance(turtlesim::Pose pos, int order[], int size) {
    double totalDist = 0;
    for(int a = 0; a < size; a++) {
        if(a == 0) {
            totalDist = find_distance(pos.x, pos.y, x_coords[order[a]], y_coords[order[a]]);
        } else {
            totalDist += find_distance(x_coords[order[a-1]], y_coords[order[a-1]], x_coords[order[a]], y_coords[order[a]]);
        }
    }
    return totalDist;
}

vector<int> find_basic_order(turtlesim::Pose pos) {
    double shortest_dist = -1;
    int size = (sizeof(x_coords)/sizeof(*x_coords));
    int order[size];
    vector<int> best_order;
    
    for(int a = 0; a < size; a++) {
        order[a] = a;
        best_order.push_back(a);
    }
    
    do {
        double total_dist = find_total_distance(pos, order, size);
        if(total_dist < shortest_dist || shortest_dist == -1) {
            shortest_dist = total_dist;
            for(int a = 0; a < size; a++) {
                best_order[a] = order[a];
            }
        }
    } while ( std::next_permutation(order,order+size) );

    cout << "Shortest distance: " << shortest_dist << endl;
    cout << "The order is: ";
    for(int a = 0; a < size; a++) {
        if(a != (size - 1)) {
            cout << best_order.at(a) << ", ";
        } else {
            cout << best_order.at(a) << endl;
        }
    }
    
    
    
    return best_order;
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
    ros::Rate r(60);

    ros::service::waitForService("kill");
    ros::ServiceClient kill_client = n.serviceClient<turtlesim::Kill>("kill");
    
    bool calculate = true;
    bool finished = false;
    int solution = 0;
    vector<int> best_order;
    
    while(ros::ok()) {

        //cout << "Current x: " << current_pose.x << endl;
        //cout << "Current y: " << current_pose.y << endl;
        
        if(first_msg && !finished) {
            if(calculate) {
                cout << "Calculating fastest route..." << endl;
                best_order = find_basic_order(current_pose);
                calculate = false;
            } else {
                
                if(at_target) {
                    
                    if(solution != 0) {
                        turtlesim::Kill srv;
                        int numTurtle = best_order.at(solution - 1) + 2;
                        std::stringstream strm;
                        strm << "turtle" << numTurtle;
                        srv.request.name = strm.str();
                        kill_client.call(srv);
                    }
                    
                    if(solution == best_order.size()) {
                        cout << "We have finished!!" << endl;
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                        velocity_pub.publish(msg);
                        finished = true;
                    } else {
                        if(next_target.empty()) {
                            next_target.push_back(x_coords[best_order.at(solution)]);
                            next_target.push_back(y_coords[best_order.at(solution)]);
                        } else {
                            next_target.at(0) = x_coords[best_order.at(solution)];
                            next_target.at(1) = y_coords[best_order.at(solution)];
                        }
                        solution++;
                        at_target = false;
                        at_target = travel_to_target(velocity_pub, current_pose, next_target);
                    }
                } else {
                    at_target = travel_to_target(velocity_pub, current_pose, next_target);
                }
            }
        }
        
		r.sleep();
		ros::spinOnce();
	}
    
    return 0;
}
