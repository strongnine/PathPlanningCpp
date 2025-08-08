#include "../Maze/maze.h"
#include "PathSearch.h"

enum HEURISTIC_TYPE {
    MANHATTAN = 1,
    EUCLIDEAN = 2,
};

// Anytime Repairing A_star
class ARA_star : public PathSearch {
 public:
    ARA_star();
    ~ARA_star();
    void init(Coordinate start, Coordinate goal, int type = 1);
    void searching(std::vector<Coordinate>& path,
                   std::vector<Coordinate>& visited_order, float* g);
    void set_e(float e);
    float get_e();
    bool is_search_done();

 private:
    void improve_path(std::vector<Coordinate>& path,
                      std::vector<Coordinate>& visited_order,
                      std::map<Coordinate, float>& open_set,
                      std::map<Coordinate, float>& incons_set);
    float calc_f_value(const Coordinate& p, const float* g, float e = 1.f);
    float heuristic(const Coordinate& p);
    float update_e(std::map<Coordinate, float>& open_set,
                   std::map<Coordinate, float>& incons_set);

    std::map<Coordinate, float> open_set_;
    std::map<Coordinate, float> incons_set_;

    int heuristic_type_ = 1;
    float e_ = 1.f;
    bool init_state_ = false;

    Coordinate* parent_;
    bool* visited_;
    float* g_;

    bool search_done_state_ = false;
};