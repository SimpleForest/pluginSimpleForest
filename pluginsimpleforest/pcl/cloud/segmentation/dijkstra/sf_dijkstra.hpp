#ifndef SF_DIJKSTRA_HPP
#define SF_DIJKSTRA_HPP

#include "sf_dijkstra.h"

#include <pcl/search/kdtree.h>

SF_Dijkstra::SF_Dijkstra( pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                          const typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInSeeds,
                          float range):
    _cloudIn(cloudIn),
    _cloudInSeeds(cloudInSeeds),
    _range(range) {
    initialize();
    compute();
}

void SF_Dijkstra::transferIntensity(){
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (_cloudInSeeds);
    size_t size = _cloudIn->points.size();
    float sqrdRange =  _range*_range;
    for(size_t i = 0; i < size; i++) {
        pcl::PointXYZI point = _cloudIn->points[i];
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            if(pointNKNSquaredDistance[0] < sqrdRange) {
                int index = _cloudInSeeds->points[pointIdxNKNSearch[0] ].intensity;
                _cloudIn->points[i].intensity = index;
                _distances.push_back(0);
            } else {
                _cloudIn->points[i].intensity = -1;
                _distances.push_back(1000);
            }
        } else {
            _cloudIn->points[i].intensity = -1;
            _distances.push_back(1000);
        }
    }
}

void SF_Dijkstra::initializeHeap(){
    _priorityQueue.clear();
    size_t size = _cloudIn->points.size();
    _points.resize(size);
    _handle.clear();
    for(size_t i = 0; i < size; i++) {
        Point point;
        point._point = _cloudIn->points[i];
        point._distance = _distances[i];
        point._visited = false;
        _points[i] = point;
        Heap::handle_type h = _priorityQueue.push(point);
        _handle.push_back(h);
        (*h).handle = h;
    }
}

void SF_Dijkstra::initializeKDTree(){
    _kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    _kdtree->setInputCloud(_cloudIn);
}

void SF_Dijkstra::initialize(){
    transferIntensity();
    initializeHeap();
    initializeKDTree();
}

int SF_Dijkstra::getIndex(const pcl::PointXYZI &point) {
    std::vector<int> indices(1);
    std::vector<float> sqrtDistances(1);
    _kdtree->nearestKSearch(point, 1, indices, sqrtDistances);
    return indices[0];
}

std::vector<int> SF_Dijkstra::getNeighbors(const pcl::PointXYZI &point) {
    std::vector<int> indices;
    std::vector<float> sqrtDistances;
    std::vector<int> result;
    _kdtree->radiusSearch(point, _range,indices, sqrtDistances);
    for(int i = 0; i < indices.size(); i++) {
        int index = indices[i];
        if(!_points[index]._visited) {
            result.push_back(index);
        }
    }
    return result;
}

float SF_Dijkstra::getDistance(const pcl::PointXYZI &p1,
                               const pcl::PointXYZI &p2) {
    float dx, dy, dz;
    dx = p1.x - p2.x;
    dy = p1.y - p2.y;
    dz = p1.z - p2.z;
    return (std::sqrt(dx*dx+dy*dy+dz*dz));
}

void SF_Dijkstra::compute() {
    bool finished = false;
    while(!finished && !_priorityQueue.empty()) {
        Point pointStruct = _priorityQueue.top()._point;
        pcl::PointXYZI point = pointStruct._point;
        int index = getIndex(point);
        if(pointStruct._distance == 1000){
            finished = true;
        } else {
            _priorityQueue.pop();
            _points[index]._visited = true;
            std::vector<int> neighbors = getNeighbors(point);
            for(int i = 0; i < neighbors.size(); i++) {
                int indexNeighbor = neighbors[i];
                Point neighbor  = (*_handle[indexNeighbor])._point;
                float distBetween = getDistance(point, neighbor._point);
                if(pointStruct._distance + distBetween < neighbor._distance) {
                    (*_handle[indexNeighbor])._point._distance = pointStruct._distance + distBetween;
                    (*_handle[indexNeighbor])._point._point.intensity = pointStruct._point.intensity;
                    _cloudIn->points[indexNeighbor].intensity = pointStruct._point.intensity;
                    _priorityQueue.decrease(_handle[indexNeighbor]);
                }
            }
        }
    }
}

#endif // SF_DIJKSTRA_HPP
