//
// Created by xiang on 2021/9/16.
//
#pragma once

#include <glog/logging.h>
#include <execution>
#include <list>
#include <thread>
#include <unordered_map>

#include "common/eigen_types.h"
#include "core/ivox3d/ivox3d_node.hpp"
#include "core/lightning_math.hpp"

namespace lightning {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6, // 6邻域
        NEARBY18, // 18邻域
        NEARBY26, // 26邻域
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    /// get statistics of the points 统计网格点分布情况
    std::vector<float> StatGridPoints() const;

   private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    // 使用 unordered_map list 组合实现LRU缓存机制
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, math::hash_vec<dim>>
        grids_map_;                                        // voxel hash map
    std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
    std::vector<KeyType> nearby_grids_;                    // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(math::ToEigen<float, dim>(pt)); // 查询点转体素网格索引
    
    // 遍历key的相邻网格，找到最邻近点候选集合
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    // 找到最接近点
    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(math::ToEigen<float, dim>(pt));

// #define INNER_TIMER
#ifdef INNER_TIMER
    static std::unordered_map<std::string, std::vector<int64_t>> stats;
    if (stats.empty()) {
        stats["knn"] = std::vector<int64_t>();
        stats["nth"] = std::vector<int64_t>();
    }
#endif

    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif

    if (candidates.size() <= max_num) {
    } else {
        // 候选点数量超过最大数量，截取前n个点
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["nth"].emplace_back(nth);

    constexpr int STAT_PERIOD = 100000;
    if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
        for (auto& it : stats) {
            const std::string& key = it.first;
            std::vector<int64_t>& stat = it.second;
            int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
            int64_t num_ = stat.size();
            stat.clear();
            std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_ << " ave=" << 1.0 * sum_ / num_
                      << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
        }
    }
#endif

    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumPoints() const {
    int ret = 0;
    for (auto& g : grids_map_) {
        ret += g.second->second.Size();
    }
    return ret;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        // 中心 + 上下左右前后六个方向
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        // 中心 + 6个轴向 + 12个面对角线方向
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        // 中心 + 6个轴向 + 12个面对角线 + 8个体对角线
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        LOG(ERROR) << "Unknown nearby_type!";
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        index[i] = i;
    }
    closest_cloud.resize(cloud.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloud, &closest_cloud, this](size_t idx) {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
            closest_cloud[idx] = pt;
        } else {
            closest_cloud[idx] = PointType();
        }
    });
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    // 根据点位置分配到相应体素网格，并维护一个LRU的缓存机制
    std::for_each(points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        // 点坐标转维对应体素网格索引key
        auto key = Pos2Grid(math::ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            // 网格索引乘分辨率得到网格中心点坐标
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            // 在缓存链表前端创建新节点
            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            // 在哈希表中记录该网格的索引和在链表中的位置
            grids_map_.insert({key, grids_cache_.begin()});
            // 将点插入到新创建的网格中
            grids_cache_.front().second.InsertPoint(pt);

            // 当网格总数达到容量上限时，删除链表末尾（最久未使用的）网格节点
            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        } else {
            // key: iter->second, 插入到网格后，移动该网格到链表前端，标记为最近使用
            iter->second->second.InsertPoint(pt);
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }
    });
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    // 点坐标除以地图分辨率
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    // 统计IVox中各个体素网格的点分布情况
    int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;
    for (auto& it : grids_cache_) {
        int s = it.second.Size(); // 每个网格中的点数
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s; // 用于计算标准差
    }
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
    return std::vector<float>{float(valid_num), ave, float(max), float(min), stddev};
}

}  // namespace lightning
