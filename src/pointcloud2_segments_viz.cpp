#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/colors.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;
std::string base_link_frame;


bool display_stationary_clusters;
visualization_msgs::Marker marker_sphere;
ros::Publisher marker_pub;

std::vector<uint> red = {0, 0, 255, 255, 255, 102, 102, 204, 0, 255};
std::vector<uint> green = {0, 255, 0, 255, 255, 102, 102, 0, 255, 152};
std::vector<uint> blue = {255, 0, 0, 0, 255, 152, 52, 152, 255, 52};


void pc2s_callback (const pointcloud_msgs::PointCloud2_Segments& msg){

    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud2 cluster_msgs;

    for (size_t i=0; i < msg.clusters.size(); i++){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        if (msg.cluster_id.size() > 0){
            uint mod = msg.cluster_id[i] % 10;

            for(size_t j=0; j < cloud.points.size(); j++){
                cloud.points[j].r = red[mod];
                cloud.points[j].g = green[mod]; 
                cloud.points[j].b = blue[mod];
            }
        }
        else {
            for(size_t j=0; j < cloud.points.size(); j++){
                uint mod = j % 10;
                cloud.points[j].r = red[mod];
                cloud.points[j].g = green[mod]; 
                cloud.points[j].b = blue[mod];      
            }
        }

        pcl::PCLPointCloud2 clouds;
        pcl::toPCLPointCloud2(cloud, clouds);
        pcl_conversions::fromPCL(clouds, cluster_msgs);
        cluster_msgs.header.stamp = ros::Time::now();
        cluster_msgs.header.frame_id = base_link_frame;

        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);

        pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);
    }


    if(display_stationary_clusters==true){


        marker_sphere.ns = "shapeOfsphere";
        marker_sphere.id = 0;
        marker_sphere.type = 7;
        marker_sphere.action = visualization_msgs::Marker::ADD;
        marker_sphere.pose.orientation.w = 1.0;
        marker_sphere.color.a = 1.0;
        marker_sphere.color.r = 1.0;
        marker_sphere.color.g = 179.000/255;
        marker_sphere.color.b = 179.000/255;


        marker_sphere.scale.x = 0.1;
        marker_sphere.scale.y = 0.1;
        marker_sphere.scale.z = 0.1;


        marker_sphere.header.frame_id = "/base_link";
        marker_sphere.header.stamp = msg.header.stamp;
        marker_sphere.lifetime = ros::Duration();


        for (size_t i=0; i < msg.stationary_clusters.size(); i++){

            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL( msg.stationary_clusters[i] , cloud2);

            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            pcl::fromPCLPointCloud2(cloud2, cloud);

            for(size_t j=0; j < cloud.points.size(); j++){
                cloud.points[j].r = 255;
                cloud.points[j].g = 179; 
                cloud.points[j].b = 179;
            }


            geometry_msgs::Point p;
                               
            p.x = cloud.points[0].x;
            p.y = cloud.points[0].y;
            p.z = cloud.points[0].z;
            marker_sphere.points.push_back(p);



            pcl::PCLPointCloud2 clouds;
            pcl::toPCLPointCloud2(cloud, clouds);
            pcl_conversions::fromPCL(clouds, cluster_msgs);
            cluster_msgs.header.stamp = ros::Time::now();
            cluster_msgs.header.frame_id = base_link_frame;

            sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);

            pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);
        }

        marker_pub.publish(marker_sphere);
        marker_sphere.points.clear();
    }

    pub.publish(accumulator);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud2_segments_viz");
    ros::NodeHandle n_;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::string input_topic;
    std::string out_topic;
    std::string stationary_topic;
    n_.param("pointcloud2_segments_viz/input_topic",input_topic, std::string("/new_pcl"));
    n_.param("pointcloud2_segments_viz/base_link_frame", base_link_frame, std::string("base_link"));
    n_.param("pointcloud2_segments_viz/out_topic", out_topic, std::string("pointcloud2_segments_viz/pointcloud2"));
    n_.param("pointcloud2_segments_viz/stationary_topic", stationary_topic, std::string("pointcloud2_clustering/stationary_clusters"));

    n_.param("pointcloud2_segments_viz/display_stationary_clusters", display_stationary_clusters, true);

    ros::Subscriber sub = n_.subscribe (input_topic, 1, pc2s_callback);
    // if(display_stationary_clusters == true) ros::Subscriber stationary_sub = n_.subscribe (stationary_topic, 1, pc2s_callback);

    pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

    if(display_stationary_clusters==true)   marker_pub = n_.advertise<visualization_msgs::Marker> ("visualization_marker_sphere", 1);

    ros::spin ();
}
