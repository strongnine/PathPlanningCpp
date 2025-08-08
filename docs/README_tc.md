<p align="center">
    <a href="../README.md">English</a>
    ·
    <a href="./README_cn.md">简体中文</a>
</p>

Overview
------
本項目實現了常用的路徑規劃算法，包括基於搜索的算法以及基於採樣的算法。該項目參考 zhm-real 的 Python 版本 [PathPlanning 項目](https://github.com/zhm-real/PathPlanning)，實現了 C++ 版本，供大家學習以及參考。

（目前只實現了部分算法，後續會繼續更新。）


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

## 動畫 - Search-Based

### Best-First & Dijkstra

<div align=right>
<table>
  <tr>
    <td><img src="../data/gif/best_first.gif" alt="dfs" width="400"/></a></td>
    <td><img src="../data/gif/dijkstra.gif" alt="dijkstra" width="400"/></a></td>
  </tr>
</table>
</div>
