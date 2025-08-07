#include "../Maze/maze.h"
#include "PathSearch.h"

enum HEURISTIC_TYPE {
    MANHATTAN = 1,
    EUCLIDEAN = 2,
};

class AStar : public PathSearch {
 public:
    AStar();
    ~AStar();
    void init(Coordinate start, Coordinate goal, int type = 1);
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, const float* g);
    void repeated_searching(std::vector<Coordinate>& path,
                            std::vector<Coordinate>& visited_order, const float* g);

    void set_e(float e);
    float get_e();

 private:
    float calc_f_value(const Coordinate& p, float* g, float e = 1.f);
    float heuristic(const Coordinate& p);

    int heuristic_type_ = 1;
    float e_ = 1.f;
};