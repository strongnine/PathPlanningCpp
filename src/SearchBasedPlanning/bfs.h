#include "../Maze/maze.h"
#include "PathSearch.h"

class BFS : public PathSearch {
 public:
    BFS();
    ~BFS();
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, float* g);

 private:

};