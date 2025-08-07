#include "../Maze/maze.h"
#include "PathSearch.h"

enum HEURISTIC_TYPE {
    MANHATTAN = 1,
    EUCLIDEAN = 2,
};

class BestFirst : public PathSearch {
 public:
    BestFirst();
    ~BestFirst();
    void init(Coordinate start, Coordinate goal, int type = 1);
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, float* g);

 private:
    float heuristic(const Coordinate& p);

    int heuristic_type_ = 1;
};