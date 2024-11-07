//自己写的聚类算法
/*
  每次传入一个点，用这个点遍历所有历史点，找到与它距离满足阈值的分组
  假设组4 6 8
  取首个组作为目标组，其余6 8 所有点添加到4组中，删除6 8 组的点
*/
	struct CMapUpload::SmartNoGoZone
	{
		vector<Point2f>allPoses;
		vector<int>label;
		std::map<int, std::vector<int>> clusters;
		std::map<int, std::vector<cv::Point2f>> clustersMap;

		cv::Point2f inputPt;
		SHARE_MAP::TShareMapPoint origin{999.9,999.9};
		float dis_thres = 0.6;
		int TRIGGER_CNT = 3;

		int ptsLen = 0;
		using ZoneType =  enum{STUCK=0,BUMPING=1};
		ZoneType zoneType;

		SmartNoGoZone(int zoneType_) { zoneType = (ZoneType)zoneType_; }

		cv::Rect2f getRect(cv::Point2f center)
		{
			float len = dis_thres * 0.5;
			return cv::Rect2f{ center.x - len, center.y - len, 2 * len, 2 * len };
		}

		void sanityCheck()
		{
			const SHARE_MAP::CMap& pMap = SHARE_MAP::CShareMap::GetInstance().GetMap();
			const SHARE_MAP::TShareMapPoint& origin_cur = pMap.base_info.point;

			//如果地图发生改变则清空所有禁区;
			//为避免地图一直不变导致内存泄漏加入size限制
			//TODO外部调用决定清空
			if ((origin.x != origin_cur.x and origin.y != origin_cur.y) 
				or ptsLen > 100000)
			{
				origin = origin_cur;
				clustersMap.clear();
				ptsLen = 0;

				clusters.clear();
				vector<int>().swap(label);
				std::vector<cv::Point2f>().swap(allPoses);
			}
		}

		void setInput(SLAM::RealtimePosition pose) 
		{
			cv::Point2f pt;
			pt.x = pose.m_PoseX;
			pt.y = pose.m_PoseY;
			ptsLen++;

			sanityCheck();

			allPoses.push_back(pt);
			ptsLen = allPoses.size();
			label.push_back(ptsLen - 1);
			inputPt = pt;

			if (zoneType == STUCK) {
				dis_thres = 0.6;
				TRIGGER_CNT = 3;
			}
			else if (zoneType == BUMPING)
			{
				dis_thres = 0.3;
				TRIGGER_CNT = 10;
			}

			build_usemap();
		}

		void build() 
		{
			if (ptsLen ==1)
				return;

			clusters.clear();
			set<int>mergeClusterIds;

			for (size_t i = 0; i < ptsLen - 1; i++)
			{
				cv::Point2f pt = allPoses[i] - inputPt;
				auto dis = std::max(std::fabs(pt.x), std::fabs(pt.y));
				if (dis < dis_thres)
				{
					mergeClusterIds.emplace(label[i]);
				}
				//clusters[label[i]].push_back(i);//方法2
			}

			if (mergeClusterIds.empty())
				return;

			label.back() = *mergeClusterIds.begin();

			//1.利用label对cluster作增删操作
			for (size_t i = 0; i < label.size(); i++)
			{
				auto it = mergeClusterIds.begin();
				if (mergeClusterIds.count(label[i]))
					clusters[*it].push_back(i);
				else
					clusters[label[i]].push_back(i);
			}

			for (auto& group : clusters)
			{
				if (!mergeClusterIds.count(group.first))
					continue;

				for (auto idx : group.second)
				{
					label[idx] = group.first;
				}
			}

			//2.直接对cluster增删
			/*clusters[label.back()].push_back(allPoses.size()-1);
			vector<int>mergeIds(mergeClusterIds.begin(), mergeClusterIds.end());
			for (size_t i = 1; i < mergeIds.size(); i++)
			{
				auto source_clus = clusters[mergeIds[i]];
				auto& target_clus = clusters[mergeIds[0]];
				target_clus.insert(target_clus.end(), source_clus.begin(), source_clus.end());
				clusters.erase(mergeIds[i]);
			}

			//更新合并后编号，label以一维存放分组信息
			//方便用vector遍历所有元素时获取可合并的分组编号，实测没有比map快，可能只是减少内存碎片
			for(auto idx: clusters[mergeIds[0]])
				label[idx] = mergeIds[0];
			*/
		}

		//3.最高效的版本:避免每次所有元素重新插入map容器，只遍历一次
		void build_usemap()
		{
			int sum = 0;
			std::set<int>mergeClusterIds;

			for (auto& group : clustersMap)
			{
				for (auto& pointf : group.second) {
					cv::Point2f pt = pointf - inputPt;
					auto dis = std::max(std::fabs(pt.x), std::fabs(pt.y));
					if (dis < dis_thres)
					{
						mergeClusterIds.emplace(group.first);
						break;
					}
				}
				sum += group.second.size();		
			}

			if (mergeClusterIds.empty()) {
				clustersMap[sum].push_back(inputPt);
				return;
			}

			//最新元素和其他组所有元素，合并到mergeIds第一位的下标
			vector<int>mergeIds(mergeClusterIds.begin(), mergeClusterIds.end());
			auto& target_clus = clustersMap[mergeIds[0]];
			target_clus.push_back(inputPt);

			for (size_t i = 1; i < mergeIds.size(); i++)
			{
				auto source_clus = clustersMap[mergeIds[i]];
				target_clus.insert(target_clus.end(), source_clus.begin(), source_clus.end());
				clustersMap.erase(mergeIds[i]);
			}
		}

		std::vector<std::vector<cv::Point2f>> Clustering() 
		{
			std::vector<std::vector<cv::Point2f>>nogoZoneCenters;
			//for (auto& clus : clusters)
			for (auto& clus : clustersMap)
			{
				if (clus.second.size() >= TRIGGER_CNT)
				{
					std::vector<cv::Point2f>nogoZonePts;
					for (size_t i = 0; i < 3; i++)
					{
						nogoZonePts.push_back(clus.second[i]);
					}
					nogoZoneCenters.push_back(nogoZonePts);
				}
			}
			return nogoZoneCenters;
		}

		std::vector<std::vector<SLAM::RealtimePosition>> getSmartNogoZones()
		{
			std::vector<std::vector<SLAM::RealtimePosition>>nogoZones;
			auto zoneCenters = Clustering();
			for (auto& zone:zoneCenters)
			{
				auto r1 = getRect(zone[0]);
				auto r2 = getRect(zone[1]);
				auto r3 = getRect(zone[2]);
				auto r = r1 & r2;	
				r &= r3;

				if (r.width < 0.05 or r.height < 0.05) 
					continue;

				cv::Point2f center;
				center.x = r.tl().x + (float)r.width * 0.5;
				center.y = r.tl().y + (float)r.height * 0.5;

				SLAM::RealtimePosition tl, bl, br, tr;
				tl.m_PoseX = center.x - dis_thres * 0.5;
				tl.m_PoseY = center.y - dis_thres * 0.5;
				bl.m_PoseX = center.x - dis_thres * 0.5;
				bl.m_PoseY = center.y + dis_thres * 0.5;
				br.m_PoseX = center.x + dis_thres * 0.5;
				br.m_PoseY = center.y + dis_thres * 0.5;
				tr.m_PoseX = center.x + dis_thres * 0.5;
				tr.m_PoseY = center.y - dis_thres * 0.5;

				std::vector<SLAM::RealtimePosition>zone_tmp{tl,bl,br,tr};
				nogoZones.push_back(zone_tmp);
			}

			return nogoZones;
		}
	};

	struct CMapUpload::SmartNOGOZone
	{
		vector<Point2f>allPoses;
		std::vector<cv::Point2f> nogoZoneCenters;
		cv::Mat zoneFilledMap;

		SHARE_MAP::TShareMapPoint origin{999.9,999.9};
		int map_h, map_w;
		float dis_thres = 0.6;
		int TRIGGER_CNT = 3;
		int gridDis_thres = 6;

		using ZoneType =  enum{STUCK=0,BUMPING=1};
		ZoneType zoneType;

		SmartNOGOZone(int zoneType_) 
		{ 
			zoneType = (ZoneType)zoneType_; 
			zoneFilledMap.create(600,600,0);
		}

		cv::Rect2f getRect(cv::Point2f center)
		{
			float len = dis_thres * 0.5;
			return cv::Rect2f{ center.x - len, center.y - len, 2 * len, 2 * len };
		}

		cv::Point Pose2Point(SLAM::RealtimePosition pose)
		{
			int row = int((map_h - std::abs(origin.x) * 20 - 0.5) - (20 * pose.m_PoseX + 0.5));
			int col = int((map_w - std::abs(origin.y) * 20 - 0.5) - (20 * pose.m_PoseY + 0.5));
			return cv::Point{col, row};
		};

		cv::Point2f Point2Pose(cv::Point pt)
		{
			float x = (((map_h - std::abs(origin.x) * 20 - 0.5) - pt.y)*5-2.5f)/100.0f;
			float y = (((map_w - std::abs(origin.y) * 20 - 0.5) - pt.x)*5-2.5f)/100.0f;
			return cv::Point2f(x,y);
		};

		void sanityCheck()
		{
			const SHARE_MAP::CMap& pMap = SHARE_MAP::CShareMap::GetInstance().GetMap();
			const SHARE_MAP::TShareMapPoint& origin_cur = pMap.base_info.point;
			map_h = pMap.base_info.height;
			map_w = pMap.base_info.width;

			//如果地图发生改变则清空所有禁区;
			//为避免地图一直不变导致内存泄漏加入size限制
			//TODO外部调用决定清空
			if ((origin.x != origin_cur.x and origin.y != origin_cur.y) 
				or allPoses.size()> 100000)
			{
				origin = origin_cur;
				std::vector<cv::Point2f>().swap(allPoses);
			}

			int sideLen = std::max(map_h, map_w);

			if (sideLen> 600)
			{
				cv::copyMakeBorder(zoneFilledMap,zoneFilledMap,200,200,200,200,BORDER_CONSTANT,0);
			}
			else if (sideLen> 1000)
			{
				int margin = (sideLen - 1000) *0.5 + 10;
				cv::copyMakeBorder(zoneFilledMap,zoneFilledMap,margin,margin,margin,margin,BORDER_CONSTANT,0);
			}
		}

		void setInput(SLAM::RealtimePosition pose) 
		{
			cv::Point2f pt;
			pt.x = pose.m_PoseX;
			pt.y = pose.m_PoseY;

			sanityCheck();

			allPoses.push_back(pt);

			if (zoneType == STUCK) {
				dis_thres = 0.6;
				TRIGGER_CNT = 3;
				gridDis_thres = 6;
			}
			else if (zoneType == BUMPING)
			{
				dis_thres = 0.3;
				TRIGGER_CNT = 10;
				gridDis_thres = 3;
			}
		}

		void FillMap()
		{
			zoneFilledMap.setTo(0);

			for(auto& pt:allPoses)
			{
				SLAM::RealtimePosition pose;

				pose.m_PoseX = pt.x;
				pose.m_PoseY = pt.y;

				cv::Point constraintLoc = Pose2Point(pose);
				cv::Point delta{gridDis_thres, gridDis_thres};
				cv::Rect rect{constraintLoc - delta, constraintLoc + delta};
				rect &= cv::Rect(0,0,zoneFilledMap.cols,zoneFilledMap.rows);
				cv::Mat roiImg = zoneFilledMap(rect);

				for (size_t i = 0; i < roiImg.rows; i++)
				{
					roiImg.row(i) += 1;
				}
			}

			cv::Mat labels,stats,centroids;
			cv::Mat zoneImg = zoneFilledMap >= TRIGGER_CNT;
			int zoneCnt = cv::connectedComponentsWithStats(zoneImg, labels,stats,centroids);

			if (zoneCnt<=1) return;
			
			std::vector<cv::Point2f>().swap(nogoZoneCenters);
			for (size_t i = 1; i < zoneCnt; i++)
			{
				int h = stats.at<int>(i,CC_STAT_HEIGHT);
				int w = stats.at<int>(i,CC_STAT_WIDTH);

				if (std::min(h,w)<2) continue;

				double x = centroids.at<double>(i,0);
				double y = centroids.at<double>(i,1);

				cv::Point2f pose = Point2Pose(cv::Point2f{x,y});
				nogoZoneCenters.push_back(std::move(pose));
			}			
		}

		std::vector<std::vector<SLAM::RealtimePosition>> getSmartNogoZones()
		{
			std::vector<std::vector<SLAM::RealtimePosition>>nogoZones;

			FillMap();

			for (auto& center:nogoZoneCenters)
			{
				auto r = getRect(center);
				center.x = r.tl().x + (float)r.width * 0.5;
				center.y = r.tl().y + (float)r.height * 0.5;

				SLAM::RealtimePosition tl, bl, br, tr;
				tl.m_PoseX = center.x - dis_thres * 0.5;
				tl.m_PoseY = center.y - dis_thres * 0.5;
				bl.m_PoseX = center.x - dis_thres * 0.5;
				bl.m_PoseY = center.y + dis_thres * 0.5;
				br.m_PoseX = center.x + dis_thres * 0.5;
				br.m_PoseY = center.y + dis_thres * 0.5;
				tr.m_PoseX = center.x + dis_thres * 0.5;
				tr.m_PoseY = center.y - dis_thres * 0.5;

				std::vector<SLAM::RealtimePosition>zone_tmp{tl,bl,br,tr};
				nogoZones.push_back(zone_tmp);
			}

			return nogoZones;
		}
	};
