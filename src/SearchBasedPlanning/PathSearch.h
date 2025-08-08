#include "../Maze/maze.h"

#ifndef PATH_SEARCH_H_
#define PATH_SEARCH_H_

typedef std::priority_queue<CoordinatePair, std::vector<CoordinatePair>,
                            std::greater<CoordinatePair>>
    CoordinatePairQueue;

class PathSearch {
 public:
    PathSearch() {};
    ~PathSearch() {};
    virtual void init(Coordinate start, Coordinate goal);
    virtual void searching(std::vector<Coordinate>& path,
                           std::vector<Coordinate>& visited_order, float* g);

 protected:
    std::vector<Coordinate> get_neighbors(const Coordinate& p, const float* g);
    float calc_cost(const Coordinate& p1, const Coordinate& p2, const float* g);
    bool is_collision(const Coordinate& p1, const Coordinate& p2,
                      const float* g);
    void extract_path(std::vector<Coordinate>& path, Coordinate* parent);

    int x_range_ = 51;
    int y_range_ = 31;

    Coordinate start_;
    Coordinate goal_;
};

#endif