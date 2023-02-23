
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

int main(){
  auto pts = getTxt();
  cluster(pts);
}
