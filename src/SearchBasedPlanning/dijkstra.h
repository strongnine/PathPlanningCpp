#include "../Maze/maze.h"
#include "PathSearch.h"

class Dijkstra : public PathSearch {
 public:
    Dijkstra();
    ~Dijkstra();
    void init(Coordinate start, Coordinate goal);
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, float* g);

 private:

};