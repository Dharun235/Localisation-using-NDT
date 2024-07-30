#include "helper.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

Eigen::Matrix4f transform2D(float theta, float xt, float yt)
{
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 0) = cos(theta);
	transform(0, 1) = -sin(theta);
	transform(1, 0) = sin(theta);
	transform(1, 1) = cos(theta);
	transform(0, 3) = xt;
	transform(1, 3) = yt;

	return transform;
}

Eigen::Matrix4f transform3D(float yaw, float pitch, float roll, float xt, float yt, float zt)
{
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 0) = cos(yaw) * cos(pitch);
	transform(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	transform(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	transform(0, 3) = xt;

	transform(1, 0) = sin(yaw) * cos(pitch);
	transform(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	transform(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	transform(1, 3) = yt;

	transform(2, 0) = -sin(pitch);
	transform(2, 1) = cos(pitch) * sin(roll);
	transform(2, 2) = cos(pitch) * cos(roll);
	transform(2, 3) = zt;

	return transform;
}

Pose getPose(Eigen::Matrix4f matrix)
{
	Pose pose(Point(matrix(0,3), matrix(1,3), matrix(2,3)), Rotate(atan2(matrix(1,0), matrix(0,0)), -asin(matrix(2,0)), atan2(matrix(2,1), matrix(2,2))));
	return pose;
}

double minDistance(const vector<Point>& points, Point p)
{
	double minDist = numeric_limits<double>::max();
	for(Point point : points)
	{
		double dist = sqrt((point.x - p.x)*(point.x - p.x) + (point.y - p.y)*(point.y - p.y) + (point.z - p.z)*(point.z - p.z));
		if(dist < minDist)
			minDist = dist;
	}
	return minDist;
}

void print2DTree(vector<Point> points, Node* node, int level, int depth, ostream& out)
{
	if(node != NULL)
	{
		for(int i = 0; i < level; i++)
		{
			out << " ";
		}
		out << "(" << points[node->id].x << "," << points[node->id].y << "," << points[node->id].z << ")" << endl;
		print2DTree(points, node->left, level + 1, depth, out);
		print2DTree(points, node->right, level + 1, depth, out);
	}
}

int main()
{
    return 0;
}
