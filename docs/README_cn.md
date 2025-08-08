<p align="center">
    <a href="../README.md">English</a>
    ·
    <a href="./README_tc.md">繁體中文</a>
</p>

Overview
------
本项目实现了常用的路径规划算法，包括基于搜索的算法和基于采样的算法。本项目参考 zhm 的 Python 版本 [PathPlanning 项目](https://github.com/zhm-real/PathPlanning)，实现了 C++ 版本，可供大家学习使用。

（目前只实现了部分算法，后续会继续更新。）

Build & Run
------

```bash
git clone https://github.com/strongnine/PathPlanningCpp.git
cd PathPlanningCpp
mkdir build && cd build
cmake ..
make -j8
./src/SearchBasedPlanning/AStar
```

## 动画 - 基于搜索的算法

### Best-First & Dijkstra

<div align=right>
<table>
  <tr>
    <td><img src="../data/gif/best_first.gif" alt="dfs" width="400"/></a></td>
    <td><img src="../data/gif/dijkstra.gif" alt="dijkstra" width="400"/></a></td>
  </tr>
</table>
</div>
