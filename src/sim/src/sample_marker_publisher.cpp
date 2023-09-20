#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectangle_marker");
	ros::NodeHandle n;
	ros::Publisher poly_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon", 10);
	ros::Publisher poly_array_pub = n.advertise<jsk_recognition_msgs::PolygonArray>("polygon_array", 10);

	int polygon_num = 25;
	// 左奥角から時計周りに定義
	double rectangles[polygon_num][4][2] = {
			{{9.125, 0.545},{9.125, 0.47},{0.0, 0.47},{0.0, 0.545}},
			{{9.125, 1.935},{9.125, 0.545},{9.05, 0.545},{9.05, 1.935}},
			{{10.205, 1.935},{10.205, 1.86},{9.125, 1.86},{9.125, 1.935}},//3番目
			{{10.205,3.01},{10.205,1.935},{10.13,1.935},{10.13,3.01}},
			{{11.355,3.01},{11.355,1.76},{11.28,1.76},{11.28,3.01}},
			{{12.6,1.835},{12.6,1.76},{11.355,1.76},{11.355,1.835}},
			{{12.6,0.825},{12.6,0.75},{11.41,0.75},{11.41,0.825}},
			{{11.485,0.75},{11.485,-3.6},{11.41,-3.6},{11.41,0.75}},//8番目
			{{10.095,-0.545},{10.095,-3.6},{10.02,-3.6},{10.02,-0.545}},
			{{10.095, -0.47},{10.095,-0.545},{0,-0.545},{0,-0.47}},
			{{9.38,-1.465},{9.38,-3.6},{9.30,-3.6},{9.30,-1.465}},
			{{9.305,-1.465},{9.305,-1.540},{8.055,-1.540},{8.055,-1.465}},
			{{8.035,-1.465},{8.035,-3.60},{7.96,-3.60},{7.96,-1.465}},
			{{7.16,-1.465},{7.16,-3.61},{7.08,-3.61},{7.08,-1.465}},
			{{7.085,-1.465},{7.085,-1.540},{6.125,-1.540},{6.125,-1.465}},//15番目
			{{6.125,-1.465},{6.125,-3.61},{6.050,-3.61},{6.050,-1.465}},
			{{5.250, -1.465},{5.250,-3.61},{5.175,-3.61},{5.175,-1.465}},
			{{5.175,-1.465},{5.175,-1.540},{4.225,-1.540},{4.225,-1.465}},
			{{4.225,-1.465},{4.225,-3.61},{4.15,-3.61},{4.15,-1.465}},//19番目
			{{3.335, -1.465},{3.335, -3.61},{3.26,-3.61},{3.26, -1.465}},
			{{3.26,-1.465},{3.26,-1.540},{2.325,-1.540},{2.325,-1.465}},
			{{2.325, -1.465},{2.325, -3.60},{2.25,-3.60},{2.25,-1.465}},
			{{1.420,-1.465},{1.420,-3.61},{1.345,-3.61},{1.345,-1.465}},
			{{1.345,-1.465},{1.345,-1.540},{0.95,-1.540},{0.95,-1.465}},
			{{0.95,-1.465},{0.95, -3.60},{0.875,-3.60},{0.875,-1.465}}
			};

	jsk_recognition_msgs::PolygonArray polygons;
	ros::Rate rate(10.0);
	while (ros::ok())
	{
		std::cout << 1+1 << std::endl;
		ros::spinOnce();
		polygons.header.frame_id = "beego/odom"; // Changed to "map"
		polygons.header.stamp = ros::Time::now();

		for (int i = 0; i < polygon_num; ++i)
		{
			geometry_msgs::PolygonStamped polygon;
			polygon.header.frame_id = "beego/odom"; // Changed to "map"
			polygon.header.stamp = ros::Time::now();
			for (int j = 0; j < 4; ++j)
			{ // There should be 5 points, not 4
				geometry_msgs::Point32 p;
				p.x = rectangles[i][j][0];
				p.y = rectangles[i][j][1];
				p.z = 0;
				polygon.polygon.points.push_back(p);
			}

			polygons.polygons.push_back(polygon);
			poly_pub.publish(polygon);
			ros::Duration(1.0).sleep(); // Optional delay
		}

		// Publish the PolygonArray after all polygons have been individually published
		poly_array_pub.publish(polygons);
	}
}
