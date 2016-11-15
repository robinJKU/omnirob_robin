/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
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
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
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

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <laser_scan_matcher.h>
#include <std_srvs/Empty.h>

ros::ServiceServer scanMatchingService;
int n_samples=1;
bool scanMatchingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
void load_reference_scan();
sensor_msgs::LaserScan::Ptr reference_scan(new sensor_msgs::LaserScan);

void load_reference_scan(){
	
	int seq,secs,nsecs;
	float angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max;
	std::string frame_id;
	std::vector <float> ranges,intensities;

	ros::param::get("/header/seq",seq);
	reference_scan->header.seq=seq;

	ros::param::get("/header/stamp/secs",secs);
	reference_scan->header.stamp.sec=secs;

	ros::param::get("/header/stamp/nsecs",nsecs);
	reference_scan->header.stamp.nsec=nsecs;

	ros::param::get("/header/frame_id",frame_id);
	reference_scan->header.frame_id=frame_id;

	ros::param::get("/angle_min",angle_min);
	reference_scan->angle_min=angle_min;

	ros::param::get("/angle_max",angle_max);
	reference_scan->angle_max=angle_max;
	
	ros::param::get("/angle_increment",angle_increment);
	reference_scan->angle_increment=angle_increment;

	ros::param::get("/time_increment",time_increment);
	reference_scan->time_increment=time_increment;

	ros::param::get("/scan_time",scan_time);
	reference_scan->scan_time=scan_time;

	ros::param::get("/range_min",range_min);
	reference_scan->range_min=range_min;

	ros::param::get("/range_max",range_max);
	reference_scan->range_max=range_max;

	ros::param::get("/ranges",ranges);
	reference_scan->ranges=ranges;

	ros::param::get("/intensities",intensities);
	reference_scan->intensities=intensities;


}


bool scanMatchingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){   

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    scan_tools::LaserScanMatcher laser_scan_matcher(nh, nh_private);
    laser_scan_matcher.set_reference_scan(reference_scan);
    laser_scan_matcher.scan_matching_sub();

    //---------------------
    while (laser_scan_matcher.cnt<n_samples){

    ros::spinOnce();

    }

    //ros::spin();
    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserScanMatcher");
  ros::NodeHandle n;
  load_reference_scan();
  scanMatchingService = n.advertiseService("scan_matching_srv", scanMatchingCallback);
  ros::Rate r(50);

  while(ros::ok){

	r.sleep();
	ros::spinOnce();

  }
  return 0;
}
