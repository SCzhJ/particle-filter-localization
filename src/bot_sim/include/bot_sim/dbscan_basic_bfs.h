#pragma once

#include <queue>
#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <tf/transform_datatypes.h>
#include <deque>
//雷达参数 theta_square = 0.000279328; //theta = 0.0167131

extern int max_radar_x;
extern int max_radar_y;
extern int min_radar_x;
extern int min_radar_y;
int obstacle_enlargement=4;
int MAXN=200;

//bfs方向数组
int dx[4] = {0, 0, 1, -1};
int dy[4] = {1, -1, 0, 0};

//dbscan 点
struct dbscan_Point {
    float x, y;
    int cluster = 0;
    bool visited = false;
    bool noise = false;
};
//整数点
struct int_Point {
    int x, y;
};

//double距离
double dbscan_distance(dbscan_Point a, dbscan_Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

//聚类函数
void dbscan(std::vector<dbscan_Point>& data, double eps, int minPts,int& clusterId) {
    clusterId=1;
    for (dbscan_Point& p : data) { 
        if (p.visited) continue;
        p.visited = true;

        std::vector<dbscan_Point*> neighbors;
        for (dbscan_Point& q : data) {
            if (dbscan_distance(p, q) <= eps) {
                neighbors.push_back(&q);
            }
        }

        if (neighbors.size() < minPts) {
            p.noise = true;
        } 
        else {
            clusterId++;
            p.cluster = clusterId;

            for (dbscan_Point* q : neighbors) {
                if (!q->visited) {
                    q->visited = true;
                    std::vector<dbscan_Point*> neighbors2;
                    for (dbscan_Point& r : data) {
                        if (dbscan_distance(*q, r) <= eps) {
                            // std::cout<<dbscan_distance(*q, r)<<"\n";
                            neighbors2.push_back(&r);
                        }
                    }
                    if (neighbors2.size() >= minPts) {
                        for (dbscan_Point* r : neighbors2) {
                            if (r->cluster == 0) {
                                r->cluster = clusterId;
                            }
                        }
                    }
                }
                if (q->cluster == 0) {
                    q->cluster = clusterId;
                }
            }
        }
    }
}

//bfs膨胀
void bfs(std::vector<dbscan_Point>& points, std::vector<std::vector<int>>& data, int width, int height) {
    std::queue<std::pair<int_Point, int>> q;
    for (dbscan_Point& p : points) {
        int tx = static_cast<int>(p.x);
        int ty = static_cast<int>(p.y);
        if (ty < 0 || ty >= height || tx < 0 || tx >= width) continue;
        data[ty][tx] = 100;  // 初始点值为100
        q.push({{tx, ty}, 100});
    }

    while (!q.empty()) {
        int_Point p = q.front().first;
        int value = q.front().second;
        q.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];

            // 检查坐标是否在地图内
            if (nx < 0 || ny >= height || ny < 0 || nx >= width) {
                continue;
                // std::cout<<"out of range\n";
            }

            // 检查这个点是否已经被访问过
            if (data[ny][nx] != 0) {
                continue;
                // std::cout<<"visited\n";
            }

            // 每次减10，直到等于0
            int next_value = std::max(0, value - 10);
            data[ny][nx] = next_value;
            if (next_value > 0) {
                q.push({{nx, ny}, next_value});
            }
        }
    }
}