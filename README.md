# Spatium
<img src="https://github.com/xRiveria/Spatium/blob/master/Documentation/StanfordDragon.png" width="45%"></img> <img src="https://github.com/xRiveria/Spatium/blob/master/Documentation/StanfordBunny.png" width="45%"></img>

A collection of spacial partitioning techniques made for personal use in real-time applications as well as coursework. 

The BVH implementations can comfortably handle up to 16000 meshes in an unevenly dense 3D scene, while the K-D Tree itself struggles at around 870000 triangles as tested with the Stanford Dragon model. I suspect that performance can be (much) further improved with additional optimizations targeted at mid-build operations which I've taken some liberty around for experimentations (such as naive rotations and sorts).

The Quadtree and Octree implementations are exceptionally simple yet effective. A search test of 10 million points yields speed improvements 1/18th the cost of a regular search. Optimizations were used
where possible (bit packing, locational codes, etc.) to save on memory usage where possible.

# Techniques - BVH

- Top Down: K-Split Points Approach with Surface Area Heuristics
- Bottom Up: Two Pass Merge Approach (Best Pair Filtering with Priority Queues, Candidate Merging)
- Incremental: Dynamic Insertion with Volume Heuristics & Self Balancing'

Facinatingly, with a two-pass approach for bottom-up building, real-time performance sometimes surpasses that of the top-down approach. I believe this could be due to the number of split points I'm sampling along each axis (100), although data locality could be distinctive factor as well.

# Techniques - K-D Tree

Using Surface Area Heuristics, we sample a set number of uniform positions within the AABB along each axis and pick the one with the lowest cost as the split point. Heavy optimizations are used here to reduce the memory usage of individual
tree nodes to improve traversal performance.

## Compilation

To build the project, simply navigate to the `Scripts` folder and run `SpatiumBuildWindows.bat`. This will leverage Premake and automatically generate a C++17 solution in the project's root directory.

# References

- [Stochastic Subsets for Bounding Volume Hierarchy (BVH) Construction](https://www.intel.cn/content/www/cn/zh/developer/articles/technical/bvh-construction.html)
- [Dynamic Bounding Volume Hierarchies by Erin Catto](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf)
- [Five Balltree Construction Algorithms by Omohundro](https://steveomohundro.com/wp-content/uploads/2009/03/omohundro89_five_balltree_construction_algorithms.pdf)
- [Real Time Collision Detection by Christer Ericson](https://realtimecollisiondetection.net)
- [Automatic Creation of Object Hierarchies for Ray Tracing](https://typeset.io/pdf/automatic-creation-of-object-hierarchies-for-ray-tracing-9eb2zp9js9.pdf)
