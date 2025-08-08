#include "../Maze/maze.h"
#include "PathSearch.h"

enum HEURISTIC_TYPE {
    MANHATTAN = 1,
    EUCLIDEAN = 2,
};

// Learning Real-Time A_star
class LRTA_star : public PathSearch {
 public:
    LRTA_star();
    ~LRTA_star();
    void init(Coordinate start, Coordinate goal, int N, int type = 1);
    void searching(std::vector<std::vector<Coordinate>>& path_iter,
                   std::vector<std::vector<Coordinate>>& visited_order_iter,
                   const float* g);

 private:
    void extract_path(std::vector<Coordinate>& path, const Coordinate start,
                      const Coordinate* parent);
    float heuristic(const Coordinate& p);
    bool A_star(std::vector<Coordinate>& path,
                std::vector<Coordinate>& visited_order, CoordinatePairQueue& pq,
                const float* g, const Coordinate cur_start, int N);
    void update_h_table();
    void extract_path_in_close_set(std::vector<Coordinate>& path,
                                   Coordinate& cur_start);
    bool is_equal(float* m1, float* m2, int n);

    int N_ = 250;  // number of expand nodes each iteration
    int heuristic_type_ = 1;

    Coordinate* parent_;
    bool* visited_;
    float* g_;
    float* h_;
    float* h_prime_;
};