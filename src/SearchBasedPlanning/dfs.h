#include "../Maze/maze.h"
#include "PathSearch.h"

class DFS : public PathSearch {
 public:
    DFS();
    ~DFS();
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, float* g);

 private:
};