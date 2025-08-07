#include <unordered_set>

#include "../Maze/maze.h"
#include "PathSearch.h"

enum HEURISTIC_TYPE {
    MANHATTAN = 1,
    EUCLIDEAN = 2,
};

class BidirectionalAStar : public PathSearch {
 public:
    BidirectionalAStar();
    ~BidirectionalAStar();
    void init(Coordinate start, Coordinate goal, int type = 1);
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order_fore,
                   std::vector<Coordinate>& visited_order_back, const float* g);

 private:
    float calc_f_value(const Coordinate& p, float* g);
    float heuristic(const Coordinate& p);
    void extract_path(std::vector<Coordinate>& path,
                      const Coordinate meet_point,
                      const Coordinate* parent_fore,
                      const Coordinate* parent_back);

    int heuristic_type_ = 1;
};