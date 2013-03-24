/*
 *  A fake /list_controllers Service
 * 
 *  The ROS Controller Manager provides the service
 *  /list_controllers
 *  which returns a list of those controllers that are loaded,
 *  and their state as 'stopped' or 'running'.
 * 
 *  e.g.
 *  controllers: ['head_traj_controller', 'left_arm_controller', 'left_hand_controller', ... ]
 *  state: ['running', 'running', 'running', ... ]
 * 
 *  This allows other nodes to query the list, then call the /switch_controller Service.
 * 
 *  
 *  The object_manipulator node from the Manipulation Pipeline calls the /list_controllers
 *  Service to check that the appropriate arm controller is running, before it attempts to
 *  grasp an object.
 *  Therefore, for robots using controllers outside the Controller Manager system, this node
 *  provides a fake Service that returns a list of user-specified controller names, and
 *  returns their state as 'running'.
 * 
 *  
 *  Author: David Butterworth
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"

#include <pr2_mechanism_msgs/ListControllers.h>
//#include <pr2_mechanism_msgs/SwitchController.h> // not required

#include <boost/lexical_cast.hpp>

ros::ServiceServer list_controllers_service_;
ros::ServiceServer switch_controller_service_;

std::vector<std::string> controller_names_;

// Callback for /list_controllers Service
bool serviceListControllers(pr2_mechanism_msgs::ListControllers::Request &req, pr2_mechanism_msgs::ListControllers::Response &resp)
{
    // Pretend to use Service request
    (void) req;

    // Initialize Service response
    resp.controllers.resize(controller_names_.size());
    resp.state.resize(controller_names_.size());

    // Return each controller's name & status 'running'
    for (unsigned char i=0; i < controller_names_.size(); i++)
    {
        resp.controllers[i] = controller_names_[i];
        resp.state[i] = "running";
        //resp.state[i] = "stopped";
    }

    ROS_DEBUG("List controllers service finished");
    //ROS_INFO("List controllers service finished");
    return true;
}

// Callback for /switch_controller Service
// (this is not implemented, because it is only used by the Manipulation Pipeline in specific situations)
/*
bool serviceSwitchController(pr2_mechanism_msgs::SwitchController::Request &req, pr2_mechanism_msgs::SwitchController::Response &resp)
{
    // Pretend to use Service request
    (void) req;

    // (void) resp;

    ROS_DEBUG("List controllers service finished");
    ROS_WARN("fake_controllers_list node received Service request /switch_controller");
    return true;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_controllers_list");

    // Controller names are specified as arguments
    if(argc >= 2)
    {
        ROS_INFO("Starting fake /list_controllers Service with %d controllers \n", (int)argc - 1 );

        // Add controller names to a list
        for (unsigned char i=1; i < (int)argc; i++) // start at index 1, argv[0] is the executable path
        {
            controller_names_.push_back( boost::lexical_cast<std::string>(argv[i]) );

            // Debug, print each name
            //printf("no %d: %s \n", i, argv[i] )
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_("~");

    // Advertise ROS Services, under the Node's namespace
    list_controllers_service_ = pnh_.advertiseService("list_controllers", serviceListControllers);
    //switch_controller_service_ = pnh_.advertiseService("switch_controller", serviceSwitchController); // not implemented

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

