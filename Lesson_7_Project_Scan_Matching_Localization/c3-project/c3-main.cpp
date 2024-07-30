#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>
#include <carla/client/Vehicle.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std;

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	} else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	} else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state)
{
	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle + response.t, 1.0f);
		} else {
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	} else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle + response.t, 1.0f);
		} else {
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);
		}
	}
	state.steer = min(max(state.steer + response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

int main()
{
	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){
		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
					pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
				}
			}
			if(pclCloud.points.size() > 5000){ 
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->removeAllShapes();
			viewer->removeAllPointClouds();
			renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 
			refresh_view = false;
		}
		new_scan = true;
		pclCloud.points.clear();

		// TODO: (Filter scan using voxel filter)
		typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

		// Creating the KdTree object for the search method of the extraction
		pcl::VoxelGrid<PointT> vg;
		vg.setInputCloud(scanCloud);
		double filterRes = 0.5;
		vg.setLeafSize(filterRes, filterRes, filterRes);
		vg.filter(*cloudFiltered);
		typename pcl::PointCloud<PointT>::Ptr scanCloudFiltered (new pcl::PointCloud<PointT>);

		// TODO: Find pose transform by using ICP or NDT matching
		// Initializing Normal Distributions Transform (NDT).
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

		// Setting scale dependent NDT parameters
		// Setting minimum transformation difference for termination condition.
		ndt.setTransformationEpsilon(0.01);
		// Setting maximum step size for More-Thuente line search.
		ndt.setStepSize(1);
		//Setting Resolution of NDT grid structure (VoxelGridCovariance).
		ndt.setResolution(1);

		// Setting max number of registration iterations.
		int iter = 30;
		ndt.setMaximumIterations(iter);

		// Setting point cloud to be aligned.
		ndt.setInputSource(cloudFiltered);
		// Setting point cloud to be aligned to.
		ndt.setInputTarget(mapCloud);

		// Calculating required rigid transform to align the input cloud to the target cloud.
		Eigen::Matrix4f initTransform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
		typename pcl::PointCloud<PointT>::Ptr cloud_ndt (new pcl::PointCloud<PointT>);
		ndt.align(*cloud_ndt, initTransform);

		// Transform scan so it aligns with ego's actual pose and render that scan
		Eigen::Matrix4f transformation_matrix = ndt.getFinalTransformation();

		double poseError = sqrt( pow(poseRef.position.x - pose.position.x, 2) + pow(poseRef.position.y - pose.position.y, 2) + pow(poseRef.position.z - pose.position.z, 2) );
		if(poseError > maxError){
			maxError = poseError;
		}
		cout << "pose error: " << poseError << ", max error: " << maxError << endl;
		
		pcl::transformPointCloud (*cloudFiltered, *scanCloudFiltered, transformation_matrix);
		viewer->removeAllPointClouds();
		renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 
		renderPointCloud(viewer, scanCloudFiltered, "scan", Color(1,0,0)); 
		viewer->spinOnce();
		
		// control vehicle
		if(cs.size() > 0){
			Accuate(cs[0], control);
			cs.erase(cs.begin());
			vehicle->ApplyControl(control);
		}
  	}
  	return 0;
}
