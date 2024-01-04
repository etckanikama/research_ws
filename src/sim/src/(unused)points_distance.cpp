#include <iostream>
#include <vector>
#include <cmath>

struct Point {
    double x, y;

    Point(double _x, double _y) : x(_x), y(_y) {}
};

double distance(double xA, double yA, double xB, double yB, double xP, double yP) {
    double APx = xP - xA;
    double APy = yP - yA;
    double ABx = xB - xA;
    double ABy = yB - yA;

    double cross_product = APx * ABy - APy * ABx;
    double length_AB = std::sqrt(ABx * ABx + ABy * ABy);
    double distance = std::abs(cross_product) / length_AB;

    return distance;
}

int main() {
    
    double WHITE_CENTERLINE_COODINATE[3][4] = {{0.0, 0.5, 9.0, 0.5}, {0.0, -0.5, 4.0, -0.5},{9.0,0.5,9.0,5.0}};
    
    std::vector<Point> point_cloud = {
        Point(0.0, 0.53),
        Point(1.0, 0.5),
        Point(2.0, 0.5),
        Point(3.0, 0.5),
        // その他の点を追加
    };

    for (const auto& point : point_cloud) {
        for (int i = 0; i < 3; ++i) {
            double xA = WHITE_CENTERLINE_COODINATE[i][2];
            double yA = WHITE_CENTERLINE_COODINATE[i][3];
            double xB = WHITE_CENTERLINE_COODINATE[i][0];
            double yB = WHITE_CENTERLINE_COODINATE[i][1];

            double result = distance(xA, yA, xB, yB, point.x, point.y);

            std::cout << "点 (" << point.x << ", " << point.y << ") と線分 " << (char)('A' + i * 2) << (char)('B' + i * 2) << " との距離は " << result << " です。" << std::endl;
        }
    }

    return 0;
}