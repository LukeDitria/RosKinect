#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/features/crh.h>
#include <visualization_msgs/MarkerArray.h>
typedef pcl::Histogram<90> CRH90;
ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  visualization_msgs::MarkerArray marker_array;

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

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (objects);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (objects);
    ec.extract (cluster_indices);

    marker_array.markers.resize(cluster_indices.size());

    int i = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          object_cluster->points.push_back (objects->points[*pit]); //*
        }
        object_cluster->width = object_cluster->points.size ();
        object_cluster->height = 1;
        object_cluster->is_dense = true;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normalEstimation.setInputCloud(object_cluster);
        normalEstimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.compute(*normals);

        // CRH estimation object.
        pcl::CRHEstimation<pcl::PointXYZ, pcl::Normal, CRH90> crh;
        crh.setInputCloud(object_cluster);
        crh.setInputNormals(normals);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*object_cluster, centroid);
        crh.setCentroid(centroid);

        marker_array.markers[i].header.frame_id = "base_link";
        marker_array.markers[i].header.stamp = ros::Time();
        marker_array.markers[i].ns = "my_namespace";
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::Marker::SPHERE;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].pose.position.x = centroid[0];
        marker_array.markers[i].pose.position.y = centroid[1];
        marker_array.markers[i].pose.position.z = centroid[2];
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;
        marker_array.markers[i].scale.x = 0.05;
        marker_array.markers[i].scale.y = 0.05;
        marker_array.markers[i].scale.z = 0.05;
        marker_array.markers[i].color.a = 1.0;
        marker_array.markers[i].color.r = 0.0;
        marker_array.markers[i].color.g = 0.1;
        marker_array.markers[i].color.b = 0.1;

        i++;
      }
    pub.publish (marker_array);
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
  pub = nh.advertise<visualization_msgs::MarkerArray> ("output", 1);

  // Spin
  ros::spin ();
}
