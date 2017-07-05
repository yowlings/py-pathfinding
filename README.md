# py-pathfinding

[![Build Status](https://travis-ci.org/qiao/PathFinding.js.svg?branch=master)](https://travis-ci.org/qiao/PathFinding.js)
[![Dependency Status](https://david-dm.org/qiao/pathfinding.js.png)](https://david-dm.org/qiao/pathfinding.js)
[![Documentation Status](https://readthedocs.org/projects/pathfindingjs/badge/)](https://readthedocs.org/projects/pathfindingjs/?badge=latest)

## Description
This is a simplest program in python that realized the 2-D path finding algorithms. Including 3 kinds algorithm, they are
* Breadth first search
* Dijkstra search
* Astar search

## Dependences
1 python 2.7
2 opencv 2.4
3 numpy
4 pyplot

## How to use?
### graph
Graph is a map that converted from an image with black point represent obstacle and white point as avaliable area. You can draw it yourself or download from web.
Here we have prepared 4 maps, where 5-9_2.png is a standard map which I draw for an office, office is a piece of part of the map; xlab401.png is my own office, and liutao.jpg is a picture of liutao that download from web, as our program has provided the functions to convert the colored image with 3 channels to gray image, you can try it as a map, but it may cost some time.
### start and goal 
Just the start point and goal point in 2D.
### algorithm
You can choose one algorithm from above 3 by changing the code line:
>d1.dijkstra()

## Reference
[Astar](http://www.redblobgames.com/pathfinding/a-star/introduction.html)
## License
MIT.
