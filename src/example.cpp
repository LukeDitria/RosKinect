#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_objects;
  pcl::fromROSMsg (*input, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients); 
  std::cout << "Segment Scene Plane" << std::endl;

  if (inliers->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
    std::cout << "Fliter out Plane, Keep object on plane" << std::endl;
     // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_inliers);

    // Retrieve the convex hull.
		pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(cloud_inliers);

    // Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

    // Prism object.
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
		prism.setInputCloud(cloud);
		prism.setInputPlanarHull(convexHull);
		// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
		// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
		prism.setHeightLimits(0.05f, 0.5f);
		pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

		prism.segment(*objectIndices);

		// Get and show all points retrieved by the hull.
		extract.setIndices(objectIndices);
		extract.filter(*objects);

    pcl::toPCLPointCloud2(*objects, cloud_objects);
    // Publish the model coefficients
    sensor_msgs::PointCloud2 ros_cloud;
    pcl_conversions::fromPCL(cloud_objects, ros_cloud);
    pub.publish (ros_cloud);
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
