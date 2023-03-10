
/*python 读取txt文件,显示点
  files = open("/home/yc/Desktop/xy1.txt")
  points = []
  for line in files:
      line = line.strip().split(' ')
      points.append([float(line[0]), float(line[1])])

  points = np.array(points, dtype=float)

  plt.plot(points[:, 0], points[:, 1], '.b')
  plt.show()
*/



vector<Point2f> getTxt(){
    fstream file("/home/yc/Desktop/xy1.txt");

    float x,y;
    vector<Point2f> vec;
    while (!file.eof()){
        file>>x>>y;
        vec.emplace_back(x,y);
    }

    return vec;
}

void cluster(vector<Point2f> &pts)
{
    map<int, vector<Point2f>> clusters;
    float dist_thresh = 0.15;

    for (const auto &curPt: pts)
    {
        vector<int> waiting_cluster_ids;//待添加,删除的点簇

        for (auto & cluster : clusters)
        {
            vector<Point2f> &m_pts = cluster.second;

            for (auto &pt: m_pts)
            {
                auto dist = cv::norm(curPt - pt);
                if (dist < dist_thresh)
                {
                    waiting_cluster_ids.emplace_back(cluster.first);
                    break;
                }
            }
        }

        //添加当前点到满足条件的点簇
        int curIndex = &curPt - &pts[0];
        if (waiting_cluster_ids.empty()) {
            clusters.emplace(curIndex, vector<Point2f>({curPt}));
            continue;
        }

        auto it = clusters.find(waiting_cluster_ids[0]);
        it->second.emplace_back(curPt);

        //添加同时删除其他族
        for (int i = 1; i < (int) waiting_cluster_ids.size(); ++i)
        {
            auto id = waiting_cluster_ids[i];
            auto it_delete = clusters.find(id);
            if (it_delete != clusters.end())
            {
                it->second.insert(it->second.end(), it_delete->second.begin(), it_delete->second.end());
                clusters.erase(it_delete);
            }
        }

    }

    cout << "cluster size: " << clusters.size() << endl;
}

/**
  ******************************************************************************
  * @author         : oswin
  * @brief          : find_p0--查找y值最小的坐标点,存在多个y相同最小值时,选x最小的值
  *                   comp极坐标比较谓词函数,按theta从小到大排序,相同角度取远的点
  ******************************************************************************
  */

//建议使用另外一个文件的find_p0函数,思路更加清晰
Point2f find_p0(std::vector<Point2f> &points) {
    double miny;
    Point minLoc;
    Point2f minPt;
    vector<Point> idx;

    Mat src((int) points.size(), 2, CV_32F, &points[0]);
    minMaxLoc(src.col(1), &miny, nullptr, &minLoc, nullptr);

    findNonZero(src.col(1) == miny, idx);
    auto cnt = (int) idx.size();
    minPt = points[minLoc.y];

    if (cnt == 1) {
        return minPt;
    }

    for (auto id: idx) {
        if (points[id.y].x <= minPt.x && points[id.y].y == minPt.y) minPt = points[id.y];
    }

    return minPt;
}

struct comp {
    bool operator()(const Point2f &p1, const Point2f &p2) {
        auto v1 = p1 - p0;
        auto v2 = p2 - p0;

        // P0排在首位
        if (p1.x == p0.x && p1.y == p0.y) return true;
        if (p2.x == p0.x && p2.y == p0.y) return false;

        auto angle1 = atan2(v1.y, v1.x);
        auto angle2 = atan2(v2.y, v2.x);

        if (angle1 < angle2) {
            return true;
        } else if (angle1 == angle2) {
            if (p1.y > p2.y) return true;
            return false;
        } else return false;
    }

    Point2f p0;
    comp(Point2f p0_) : p0(p0_) {}
};

/**
  ******************************************************************************
  * @author         : oswin
  * @brief          : 从输入点pt向右作射线,与多边形交点个数为奇数时,pt在其内部,否则在外部
  *                   叉乘 cross(p1 p2 pt(p3))结果为正,代表射线右侧存在一个交点;
  ******************************************************************************
  */

int pointPolygonTest(vector<Point> &pts, Point ip) {

    int counter{0};
    Point v0;
    Point v = pts.back();
    for (auto pt: pts) {
        v0 = v;
        v = pt;
        
        //不满足这些条件的点生成一条向右的射线时候,该射线无法与这个折线有交点
        if ((v0.y <= ip.y && v.y <= ip.y) ||
            (v0.y > ip.y && v.y > ip.y) ||
            (v0.x < ip.x && v.x < ip.x)) {
            if (ip.y == v.y && (ip.x == v.x || (ip.y == v0.y &&
                                                ((v0.x <= ip.x && ip.x <= v.x) || (v.x <= ip.x && ip.x <= v0.x)))))
                return 0;
            continue;
        }

        int64 dist = static_cast<int64>(ip.y - v0.y) * (v.x - v0.x)
                     - static_cast<int64>(ip.x - v0.x) * (v.y - v0.y);
      
        //int dist = (v - v0).cross(ip - v);//叉乘可用公式

        if (dist == 0)
            return 0;
        if (v.y < v0.y)
            dist = -dist;
        counter += dist > 0 ? 1 : 0;
    }

    auto result = counter % 2 == 0 ? -1 : 1;
    return result;
}

/**
  ******************************************************************************
  * @author         : oswin
  * @brief          : graham扫描线求凸包算法--输入按极坐标排序的点,输出凸包到栈
  ******************************************************************************
  */
void intoStack(std::stack<Point2f> &Stack, Point2f &p) {
    //获取栈最后两个元素
    auto prev1 = Stack.top();
    Stack.pop();
    auto prev2 = Stack.top();
    Stack.emplace(prev1);

    //判断p是否要插入栈;
    if ((prev1 - prev2).cross(p - prev1) < 0) {
        if (Stack.size() == 2) return;
        Stack.pop();//p右拐,尾元素弹出
        intoStack(Stack, p);//弹出后,递归找到末两位与p构成左拐的元素
    } else {
        Stack.emplace(p);
    }
}

void find_convex_hull(std::vector<Point2f> &points, Point2f pFirst) {
    if (points.size() < 3) printf("Points not sufficient!");

    Point2f P0 = points[0];
    Point2f P1 = points[1];

    std::stack<Point2f> ptStack;
    ptStack.emplace(P0);
    ptStack.emplace(P1);

    for (int i = 2; i < (int) points.size(); ++i) {
        if ((points[i - 1].y - pFirst.y) * (points[i].x - pFirst.x) ==
            (points[i - 1].x - pFirst.x) * (points[i].y - pFirst.y)) {
            i++;
            continue;
        }

        auto p = points[i];
        intoStack(ptStack, p);
    }

    //visualize
    {
        deque<Point2f> vec;
        while (!ptStack.empty()) {
            auto top = ptStack.top();
            ptStack.pop();
            vec.push_front(top);
        }

        Mat src((int) vec.size(), 2, CV_32F, &vec[0]);
        Mat dst(300, 300, 0);
        normalize(src, src, 0, 255, NORM_MINMAX);
        src.convertTo(src, CV_32S);

        Point last = Point(src.at<int>(0, 0), src.at<int>(0, 1));
        for (int i = 0; i < vec.size(); ++i) {
            auto p = Point(src.at<int>(i, 0), src.at<int>(i, 1));
            line(dst, last, p, 255);
            last = p;
        }
        namedWindow("dst", 2);
        imshow("dst", dst);
        waitKey();
    }
}

/**
  ******************************************************************************
  * @author         : oswin 这个算法速度最快
  * @brief          : 仿照opencv的sklansky算法,用指针(下标)方式实现求graham扫描线凸包
  
                     stack{..., prev(13),cur(37)} next(38), stacksize是stack指针
                     prev,cur,next是points的指针. 
                     模拟栈的功能:stacksize--左移(类似pop出栈),stack[stacksize] = next
                     类似入栈.
  ******************************************************************************
  */
void Sklansky(vector<Point2f> &points) {

    int total = (int) points.size();
    vector<Point2f*> _pointer(total);
    vector<int> stack_(total, 0);
    Point2f **array = (Point2f **) &_pointer[0];
    int *stack = &stack_[0];

    for (int i = 0; i < total; ++i) {
        array[i] = &points[i];
    }

    int prev, cur, next;
    stack[0] = prev = 0;
    stack[1] = cur = 1;
    next = 2;

    int end = total;
    int stacksize = 2;

    while (next != end) {
        auto v1 = array[cur][0] - array[prev][0];
        auto v2 = array[next][0] - array[cur][0];
        auto sign = v1.cross(v2);

        if (sign > 0) {
            stack[stacksize] = next;
            stacksize++;
            prev = cur;
            cur = next;
            next++;
        } else {
            stacksize--;
            cur = prev;
            prev = stack[stacksize - 2];
        }
    }
}



int main(){
  auto pts = getTxt();
  cluster(pts);
  auto p0 = find_p0(pts);
  sort(pts.begin(),pts.end(),comp(p0));
  find_convex_hull(pts,p0);
}
