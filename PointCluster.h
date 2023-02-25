//
// Created by wdh on 2023/2/17.
//

#ifndef _POINT_CLUSTER_H
#define _POINT_CLUSTER_H
#include <vector>
#include <list>

/******************************************************
 * 接口调用方式
 * pc::PointCluster *tmp = new pc::PointCluster(0.15); // 初始化对象， dist表示分簇的距离阈值
 * for (auto pt : points) // 每识别到一个就可以放入一次分簇函数
 * {
 *      pc::Point ppt(pt.x, pt.y);  // 将坐标点变为目标点的格式
 *      int id = tmp->point_cluster(pt); // 将目标点送入分簇函数中，返回的id表示当前点所在簇的索引，若为-1表示当前点为独点
 *      auto ttt =  tmp->get_cluster_contours(); // 获取所有的簇点集的凸包，每簇的点数为1表示独点，可以根据 *(ttt.begin()+id) 获取当前点所在的凸包，凸包的点集是逆时针排序
 *  }
 *
 ******************************************************/

namespace pc {
// 点的保存形式
    struct Point
    {
        float x, y; // xy为坐标系
        int id;  // 当前点属于的类别，默认为-1，表示不属于任一簇
        Point(float a, float b) : x(a), y(b), id(-1) {} // 有参数的构造
    };

    class PointCluster
    {
    public:
        explicit PointCluster(float dist = 0.15); // 初始化分簇的距离dist，只有点与点的距离满足要求，才放入一簇
        ~PointCluster();  // 释放 total_points 和 cluster_contours 容器的内存
        int point_cluster(Point pt);  // 将传入的点放入对应的簇，并返回当前点位于list中的位置，当返回-1时，表示当前点是一个独点
        std::list<std::vector<Point>> get_cluster_contours(); // 返回所有簇，一般认为一个有效的簇至少有2个点

    private:
        std::vector<Point> total_points;  // 保存所有的点
        std::list<std::vector<Point>> cluster_contours; // 保存每个簇的外轮廓点，使用list主要是为了提高元素的删除的效率
        float dist_thresh; // 分簇的距离

        /****************寻找点集的凸包**********************/
        // 参考 https://blog.csdn.net/weixin_38442390/article/details/109235183
        static void find_p0(std::vector<Point> &points);  // 寻找凸包的P0点
        static bool cmp(Point &p1, Point &p2); // 极角排序
        static inline double cross(Point &p1, Point &p2, Point &p3) // 叉积
        {
            return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        }

        void find_convex_hull(std::vector<Point> &points); // 寻找凸包所有点集
        /****************寻找点与凸包（轮廓）的位置关系，参考 opencv中 pointPolygonTest函数**********************/
        static int pointPolygonTest(std::vector<Point> &points, Point pt); // 计算当前前点是否在对应的原有凸包内

    };
}

#endif //_POINT_CLUSTER_H

