#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// std::ofstream ofs("2023-2-28_v_1.0_odom_zure_tyokusen.csv");
geometry_msgs::Quaternion robot_r;
geometry_msgs::Twist cmd_vel;

double robot_x = 0.0;
double robot_y = 0.0;

// // odomの位置(ｘ,y,theta)をsub
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;
}

// ファイル読み込み
// 引数で配列のポインタを受け取りそこに値を書き込み
int fileread(double test[], string fname) {
	ifstream fin(fname);
	if (!fin) {							
		cout << "ファイルをオープンできませんでした。\n";
		getchar();		
		exit(0);		// プログラムの終了
	}
	else cout << "ファイルをオープンしました。\n";

	// eofが検出されるまで配列に書き込む
	int i = 1;
	while (1) {
		fin >> test[i];
		if (fin.eof())	break;
		i++;
	}
	fin.close();
	cout << "ファイルをクローズしました。\n";

	return i;	                // 配列の長さを返す
}


// rolll, pitch, yawからクォータニオンにする関数

// メインの処理
int main(int argc, char **argv)
{


	const int num = 700;		// 配列の初期化サイズ
	double test[num];			// 配列初期化
	int flen;			// 読み込むデータサイズの変数
	string fname = "/home/hirayama-d/research_ws/vel_trap_p10.0v0.5a1.0.csv";	// 読み込むファイル名
	flen = fileread(test, fname);	// ファイル読み込み関数実行


    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    //cmd_velのpublisher
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // odomのsubscribe
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, odomCallback);
    ros::Rate rate(50.0);

	// 読み込めているか確認
	// for (int i = 1; i <= flen; i++) {	
	// 	cout << test[i] << endl;
	// }

	// コンソール画面が消えないように
	// getchar();
	// return 0;
    int index = 1;
    int cnt = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        // printf("robot_x, robot_y, robot_r: %f, %f, %f", robot_x, robot_y, tf::getYaw(robot_r));
        // printf("cnt, robot_x: %d,  %f\n", cnt ,robot_x);
        // printf("vel:  %f\n", test[10]);
        cmd_vel.linear.x = test[index];
        // 実際の自己位置をcsvに出力
        printf("robt_x, cnt, vel:%f %d %f\n",robot_x, cnt, cmd_vel.linear.x);
        // ofs << robot_x << ", " << robot_y << std::endl;
        // if (cnt==10){
        //     break;
        // }
        if (index == 601){
            cout <<  robot_x << " 10m超えました" << endl;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            return 0;
        }

        cmd_pub.publish(cmd_vel);
        rate.sleep();

        cnt++;
        index++;    
    }
    return 0;

    




}
