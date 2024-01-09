#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectangle_marker_node");
	ros::NodeHandle n;
	ros::Publisher poly_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon", 10);
	ros::Publisher poly_array_pub = n.advertise<jsk_recognition_msgs::PolygonArray>("polygon_array", 10);

	std::cout << "gazebo用の白線地図" << std::endl;
	
	int polygon_num = 37; //もともとのひげの数:25,1.5のひげの数:40, 統合ひげ:37
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
			{{1.345,-1.465},{1.345,-1.540},{0.545,-1.540},{0.545,-1.465}},
			{{0.545,-1.465},{0.545, -3.60},{0.47,-3.60},{0.47,-1.465}},
// {{8.761401098901098, 0.8075000000000001}, {8.761401098901098, 0.20750000000000007}, {8.686401098901099, 0.20750000000000007}, {8.686401098901099, 0.8075000000000001}}, {{7.2572802197802195, 0.8075000000000001}, {7.2572802197802195, 0.20750000000000007}, {7.182280219780221, 0.20750000000000007}, {7.182280219780221, 0.8075000000000001}}, {{3.2462912087912086, 0.8075000000000001}, {3.2462912087912086, 0.20750000000000007}, {3.171291208791209, 0.20750000000000007}, {3.171291208791209, 0.8075000000000001}}, {{5.79165, -0.20750000000000007}, {5.79165, -0.8075000000000001}, {5.7166500000000005, -0.8075000000000001}, {5.7166500000000005, -0.20750000000000007}}, {{4.378349999999999, -0.20750000000000007}, {4.378349999999999, -0.8075000000000001}, {4.30335, -0.8075000000000001}, {4.30335, -0.20750000000000007}}, {{2.3593500000000005, -0.20750000000000007}, {2.3593500000000005, -0.8075000000000001}, {2.2843500000000003, -0.8075000000000001}, {2.2843500000000003, -0.20750000000000007}}, {{11.7475, 0.1805232558139536}, {11.7475, 0.10552325581395358}, {11.1475, 0.10552325581395358}, {11.1475, 0.1805232558139536}}, {{10.357500000000002, -2.4423333333333335}, {10.357500000000002, -2.5173333333333336}, {9.7575, -2.5173333333333336}, {9.7575, -2.4423333333333335}}, {{10.357500000000002, -3.5625}, {10.357500000000002, -3.6375}, {9.7575, -3.6375}, {9.7575, -3.5625}}
// // 統合ひげ
{{4.249038461538461, 0.8075000000000001}, {4.249038461538461, 0.20750000000000007}, {4.174038461538462, 0.20750000000000007}, {4.174038461538462, 0.8075000000000001}}, {{1.341071428571429, 0.8075000000000001}, {1.341071428571429, 0.20750000000000007}, {1.2660714285714287, 0.20750000000000007}, {1.2660714285714287, 0.8075000000000001}}, {{0.0375, 0.8075000000000001}, {0.0375, 0.20750000000000007}, {-0.0375, 0.20750000000000007}, {-0.0375, 0.8075000000000001}}, {{9.4865, 2.1975}, {9.4865, 1.5975}, {9.4115, 1.5975}, {9.4115, 2.1975}}, {{8.61825, -0.20750000000000007}, {8.61825, -0.8075000000000001}, {8.54325, -0.8075000000000001}, {8.54325, -0.20750000000000007}}, {{7.305899999999999, -0.20750000000000007}, {7.305899999999999, -0.8075000000000001}, {7.2309, -0.8075000000000001}, {7.2309, -0.20750000000000007}}, {{5.79165, -0.20750000000000007}, {5.79165, -0.8075000000000001}, {5.7166500000000005, -0.8075000000000001}, {5.7166500000000005, -0.20750000000000007}}, {{2.76315, -0.20750000000000007}, {2.76315, -0.8075000000000001}, {2.68815, -0.8075000000000001}, {2.68815, -0.20750000000000007}}, {{11.7475, -0.12296511627906978}, {11.7475, -0.1979651162790698}, {11.1475, -0.1979651162790698}, {11.1475, -0.12296511627906978}}, {{11.7475, -2.6520348837209298}, {11.7475, -2.72703488372093}, {11.1475, -2.72703488372093}, {11.1475, -2.6520348837209298}}, {{10.357500000000002, -1.4240000000000002}, {10.357500000000002, -1.499}, {9.7575, -1.499}, {9.7575, -1.4240000000000002}}, {{10.357500000000002, -3.5625}, {10.357500000000002, -3.6375}, {9.7575, -3.6375}, {9.7575, -3.5625}}
// 俺が作った1.5のひげ
// {{10.3575, -3.5075}, {10.3575, -3.5825}, {9.757499999999999, -3.5825}, {9.757499999999999, -3.5075}}, {{3.0375, 0.8074999999999999}, {3.0375, 0.20749999999999996}, {2.9625, 0.20749999999999996}, {2.9625, 0.8074999999999999}}, {{6.0375, -0.20749999999999996}, {6.0375, -0.8074999999999999}, {5.9625, -0.8074999999999999}, {5.9625, -0.20749999999999996}}, {{3.0375, -0.20749999999999996}, {3.0375, -0.8074999999999999}, {2.9625, -0.8074999999999999}, {2.9625, -0.20749999999999996}}, {{7.5375, -0.20749999999999996}, {7.5375, -0.8074999999999999}, {7.4625, -0.8074999999999999}, {7.4625, -0.20749999999999996}}, {{1.5375, 0.8074999999999999}, {1.5375, 0.20749999999999996}, {1.4625, 0.20749999999999996}, {1.4625, 0.8074999999999999}}, {{1.5375, -0.20749999999999996}, {1.5375, -0.8074999999999999}, {1.4625, -0.8074999999999999}, {1.4625, -0.20749999999999996}}, {{11.775, -0.7125}, {11.775, -0.7875}, {11.175, -0.7875}, {11.175, -0.7125}}, {{11.775, -2.2125}, {11.775, -2.2875}, {11.175, -2.2875}, {11.175, -2.2125}}, {{10.3575, -2.0075}, {10.3575, -2.0825}, {9.757499999999999, -2.0825}, {9.757499999999999, -2.0075}}, {{9.0375, -0.20749999999999996}, {9.0375, -0.8074999999999999}, {8.9625, -0.8074999999999999}, {8.9625, -0.20749999999999996}}, {{6.0375, 0.8074999999999999}, {6.0375, 0.20749999999999996}, {5.9625, 0.20749999999999996}, {5.9625, 0.8074999999999999}}, {{7.5375, 0.8074999999999999}, {7.5375, 0.20749999999999996}, {7.4625, 0.20749999999999996}, {7.4625, 0.8074999999999999}}, {{4.5375, 0.8074999999999999}, {4.5375, 0.20749999999999996}, {4.4625, 0.20749999999999996}, {4.4625, 0.8074999999999999}}, {{4.5375, -0.20749999999999996}, {4.5375, -0.8074999999999999}, {4.4625, -0.8074999999999999}, {4.4625, -0.20749999999999996}}

			};
			
	jsk_recognition_msgs::PolygonArray polygons;
	for (int i = 0; i < polygon_num; ++i)
	{
		geometry_msgs::PolygonStamped polygon;
		polygon.header.frame_id = "map"; // Changed to "map" or beego/odom
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
		
	}
	ros::Rate rate(10.0);
	while (ros::ok())
	{
		
		ros::spinOnce();
		polygons.header.frame_id = "map"; // Changed to "map" or beego/odom
		polygons.header.stamp = ros::Time::now();


		

		// ros::Duration(1.0).sleep(); // Optional delay
		// Publish the PolygonArray after all polygons have been individually published
		poly_array_pub.publish(polygons);
	}
}
