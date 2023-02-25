//
// Created by wdh on 2023/2/17.
//

#include "PointCluster.h"
#include <stack>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>

namespace pc
{
    static Point P0(0.0, 0.0); // 计算凸包时，得到的点集的右下角点，作为凸包的起点
    /**
     * 构造分簇的构造函数，通过外部设定分簇的距离阈值
     * @param dist 分簇的距离阈值，小于的则为同一簇
     */
    PointCluster::PointCluster(float dist)
    {
        dist_thresh = dist;
    }

    /**
     * 析构函数，主要对保存的历史点集 total_points 和 分簇集合进行内存释放
     */
    PointCluster::~PointCluster()
    {
        std::vector<Point>().swap(total_points);
        while (!cluster_contours.empty())
        {
            std::vector<Point>().swap(cluster_contours.front()); // 对点簇内点集进行释放
            cluster_contours.pop_front(); // 弹出 点簇，达到对list的内存释放
        }
    }

    /**
     * 获取历史点簇集合
     * @return 返回历史点簇集合
     */
    std::list<std::vector<Point>> PointCluster::get_cluster_contours()
    {
        return cluster_contours;
    }

    /**
     * 将输入的点进行分簇，并根据当前点与历史所有点的距离关系，对历史簇进行重新分簇
     * @param pt 当前点
     * @return 返回当前点所在簇集合的索引，-1表示当前点是独点，
     */
    int PointCluster::point_cluster(Point pt)
    {
        int pt_id = -1;  // 当前点位于簇的索引，-1表示当前点不在簇中，即独点
        pt.id = (int) total_points.size();  // 当前点的编号暂时赋值为点数大小，后面根据分簇的结果再重新赋值
        if (total_points.empty())
        {
            total_points.push_back(pt);
            std::vector<Point> ppt;
            ppt.push_back(pt);
            cluster_contours.push_back(std::move(ppt));
            return pt_id;
        }

        // 找出满足距离要求的所有点，目标采用这样暴力遍历的方式，后续可以更改为动态KDTree，提高查询的速度
        std::vector<int> ids;  // 记录满足距离要求的点的编号
        for (auto &ppt: total_points)
        {
            float dist = std::hypot(ppt.x - pt.x, ppt.y - pt.y);
            if (dist < dist_thresh) // 距离满足的，保存点编号，即作为同一簇
            {
//                if (!ids.empty()) ppt.id = ids[0]; // 保持同一簇的编号相同
                if (std::find(ids.begin(), ids.end(), ppt.id) == ids.end()) ids.push_back(ppt.id); // 确保传入的编号不重复
            }
        }

        if (ids.empty()) // 没有找到满足距离阈值的点，则说明是独点
        {
            std::vector<Point> ppt;
            ppt.push_back(pt);
            cluster_contours.push_back(std::move(ppt));
        }
        else
        {
            // 将所有满足距离要求的点的编号进行统一规划，即将其他点的编号变成ids[0]编号相同
            for (auto i : ids)
            {
                if (pt.id > i) pt.id = i;
            }

            if (ids.size() > 1)
            {
                // 更新簇轮廓的点集
                std::vector<std::list<std::vector<Point>>::iterator> ids_iter;
                std::list<std::vector<Point>>::iterator target_iter;
                for (auto iter = cluster_contours.begin(); iter != cluster_contours.end(); iter++)
                {
                    //ids = 164,235
                    if (std::find(ids.begin(), ids.end(), (*iter)[0].id) != ids.end())
                    {
                        if ((*iter)[0].id == pt.id)
                        {
                            target_iter = iter;
                        }
                        else
                        {
                            ids_iter.push_back(iter);//ids_iter待删除的点集,存放的是list iterator
                        }
                    }
                }
                // 将其他编号下的点集，复制到目标编号的点集中
                for (auto &iter: ids_iter)
                {
                    for (auto &ppt: total_points)
                    {
                        if (ppt.id == (*iter)[0].id) ppt.id = target_iter->front().id; // 所有待合并的簇，需将簇内所有点都需要进行编号与合并的簇的编号进行统一
                    }
                    target_iter->insert(target_iter->end(), iter->begin(), iter->end()); // 深拷贝
                }
                // 删除其他编号的点集
                for (int i = (int)ids_iter.size()-1; i > -1; i--)
                {
                    auto iter = ids_iter[i];
                    std::vector<Point>().swap(*iter);
                    cluster_contours.erase(iter);
                }
            }
            std::list<std::vector<Point>>::iterator target_iter;
            bool is_update = true;
            // 将当前点插入到对应编号的点集中
            for (auto iter = cluster_contours.begin(); iter != cluster_contours.end(); iter++)
            {
                if ((*iter)[0].id == pt.id)
                {
                    pt_id = std::distance(cluster_contours.begin(), iter);

                    // 这个时候需要判断当前加入的点是否需要进行凸包计算，减少计算量
                    // 在凸包内，则不需要再计算凸包，且不用将当前点加入到点集中
                    if (ids.size() == 1 && (*iter).size() > 2)
                    {
                        int ret = pointPolygonTest(*iter, pt); // 判断pt点与凸包轮廓的位置关系, 1在轮廓内，0在轮廓上，-1在轮廓外
                        if (ret > -1) // 不在轮廓外的点不需要进行凸包更新
                        {
                            is_update = false; // 说明当前前点在在凸包轮廓内，不需要进行凸包更新
                            break;
                        }
                    }
                    (*iter).push_back(pt);
                    target_iter = iter;
                    break;
                }
            }

            // 更换了目标编号的点集，则需要更新当前点集的凸包
            if (is_update && (*target_iter).size() > 3) // 只有点集大于三个点时，才会进行凸包计算
            {
                find_convex_hull(*target_iter);
            }
        }

        total_points.push_back(pt); // 将当前点加入到总点集中
        return pt_id;
    }

    /**
     * 找到当前点集中y值最小，x值最小的点，作为凸包的起始点
     * @param points 当前的点集，对点集进行凸包查找
     * 返回找到的起始点P0
     */
    void PointCluster::find_p0(std::vector<Point> &points)
    {
        Point pp = points[0];
        for (int i = 1; i < (int) points.size(); i++)
        {
            points[i].id = pp.id;  // 保证当前点集的编号与第0个点的编号保持一致
            if (points[i].y < pp.y)  // Y轴最小的点
            {
                pp = points[i];
            }
            else if (points[i].y == pp.y) // X轴最小的点
            {
                if (points[i].x < pp.x)
                {
                    pp = points[i];
                }
            }
        }
        P0 = pp;
    }

    /**
     * 对 点p1 和 点p2 分别与 起始点P0 的极角 进行大小判断
     * @param p1 点1
     * @param p2 点2
     * @return 点2与P0的极角 大于 点1与P0的极角，则返回true，否则返回false
     */
    bool PointCluster::cmp(Point &p1, Point &p2)
    {
        // P0排在首位
        if (p1.x == P0.x && p1.y == P0.y) return true;
        if (p2.x == P0.x && p2.y == P0.y) return false;

        const double MAXNUM = 1e10;

        //计算极角（等于0则赋予一个极大值）
        double angle1 = p1.x == P0.x ? MAXNUM : (p1.y - P0.y) / (p1.x - P0.x);
        double angle2 = p2.x == P0.x ? MAXNUM : (p2.y - P0.y) / (p2.x - P0.x);
        //小于0则赋予一个更大的值
        if (angle1 < 0)angle1 += 2 * MAXNUM;
        if (angle2 < 0)angle2 += 2 * MAXNUM;

        //极角排序
        if (angle1 < angle2) return true;
        else if (angle1 == angle2)
        {
            if (p1.y > p2.y) return true;
            else return false;
        }
        else return false;
    }

    /**
     * 寻找点集的凸包
     * @param points 传入点集，凸包的点集也是用points保存
     */
    void PointCluster::find_convex_hull(std::vector<Point> &points)
    {
        find_p0(points); // 寻找 点集中 最右下角的点 作为P0
        std::sort(points.begin(), points.end(), cmp); // 对点集按照极角排序
        std::stack<Point> convex_hull;  // 采用堆栈的数据结构，保存凸包数据
        //p0和p1是凸包中的点
        convex_hull.push(points[0]);
        convex_hull.push(points[1]);

        int i = 2;
        //p1,p2为栈顶两个节点
        Point p1 = points[0];
        Point p2 = points[1];
        while (i < points.size())
        {
            //如果points[i]和points[i-1]在同一个角度，则不再对points[i]进行计算
            if ((points[i - 1].y - P0.y) * (points[i].x - P0.x) == (points[i - 1].x - P0.x) * (points[i].y - P0.y))
            {
                i++;
                continue;
            }

            //如果叉积大于0，将当前点压入栈
            if (cross(p1, p2, points[i]) >= 0)
            {
                //假设现在栈中为a,b,c,d,cross(c,d,e)大于等于0
                convex_hull.push(points[i]);//a,b,c,d,e,p1=c,p2=d
                p1 = p2;//p1=d
                p2 = convex_hull.top();//p2=e
                i++;
            }
                //如果叉积小于0，对栈中节点进行处理
            else
            {
                while (true)
                {
                    //假设现在栈中为a,b,c,d,cross(c,d,e)小于0
                    convex_hull.pop();//a,b,c
                    convex_hull.pop();//a,b
                    p2 = p1;//p2=c;
                    p1 = convex_hull.top();//p1=b
                    convex_hull.push(p2);//a,b,c
                    //cross(b,c,e)
                    if (cross(p1, p2, points[i]) >= 0) {
                        convex_hull.push(points[i]);//a,b,c,e
                        p1 = p2;//p1=c
                        p2 = convex_hull.top();//p2=e
                        i++;
                        break;
                    }
                }
            }
        }

        if (convex_hull.size() < 3 || convex_hull.size() == points.size())
        {
            // swap相当于交换了convex_hull和一个空临时stack的内容，
            // 然后临时stack再结束生命周期，但由于操作的是堆空间，其实还是一个一个释放空间。
            std::stack<Point>().swap(convex_hull); // 释放堆stack内存，比pop速度快
            return;
        }
        // 更新 凸包点
        points.clear();
        while (!convex_hull.empty())
        {
            Point pt = convex_hull.top();  // 获取堆栈中首位的数据
            points.push_back(pt); // 将首位数据保存
            convex_hull.pop(); // 并将堆栈的首位数据弹出，即释放内存
        }
    }

    /**
    * 采用opencv中的 pointPolygonTest 函数来判断点与轮廓的位置关系
    * @param points 轮廓点集，且轮廓点集必须是按照顺时针或者逆时针排序的
    * @param pt 当前点
    * @return 返回位置关系，-1表示在轮廓外，0表示在轮廓上，1表示在轮廓上
    */
    int PointCluster::pointPolygonTest(std::vector<Point> &points, Point pt)
    {
        // 为了提供计算速率，将所有数据变为整数进行计算
        int ip_x = int(pt.x * 1000.0f);
        int ip_y = int(pt.y * 1000.0f);

        int total = (int) points.size();
        int v0_x, v0_y;
        int v_x = int(points.back().x * 1000.0f);
        int v_y = int(points.back().y * 1000.0f);

        int counter = 0;
        for (int i = 0; i < total; i++)
        {
            int dist;
            v0_x = v_x;
            v0_y = v_y;

            v_x = int(points[i].x * 1000.0f);
            v_y = int(points[i].y * 1000.0f);

            if ((v0_y <= ip_y && v_y <= ip_y) ||
                (v0_y > ip_y && v_y > ip_y) ||
                (v0_x < ip_x && v_x < ip_x))
            {
                if (ip_y == v_y && (ip_x == v_x || (ip_y == v0_y &&
                                                    ((v0_x <= ip_x && ip_x <= v_x) || (v_x <= ip_x && ip_x <= v0_x)))))
                {
                    return 0;
                }
                continue;
            }

            dist = (ip_y - v0_y) * (v_x - v0_x) - (ip_x - v0_x) * (v_y - v0_y);
            if (dist == 0)
            {
                return 0;
            }
            if (v_y < v0_y)
            {
                dist = -dist;
            }
            counter += dist > 0 ? 1 : 0;
        }
        return counter % 2 == 0 ? -1 : 1;
    }

}


