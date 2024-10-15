# Spatium
<img src="https://github.com/xRiveria/Spatium/blob/master/Documentation/StanfordDragon.png" width="40%"></img> <img src="https://github.com/xRiveria/Spatium/blob/master/Documentation/StanfordBunny.png" width="40%"></img>

A collection of space partioning techniques made for personal use in real-time applications as well as coursework. 

# Techniques - BVH

- Top Down: K-Split Points Approach with Surface Area Heuristics
- Bottom Up: Two Pass Merge Approach (Best Pair Filtering with Priority Queues, Candidate Merging)
- Incremental: Dynamic Insertion with Volume Heuristics & Self Balancing

## Compilation

To build the project, simply navigate to the `Scripts` folder and run `SpatiumBuildWindows.bat`. This will leverage Premake and automatically generate a C++17 solution in the project's root directory.

# References

- [Dynamic Bounding Volume Hierarchies by Erin Catto](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf)
- [Five Balltree Construction Algorithms by Omohundro](https://steveomohundro.com/wp-content/uploads/2009/03/omohundro89_five_balltree_construction_algorithms.pdf)
- [Real Time Collision Detection by Christer Ericson](https://realtimecollisiondetection.net)
- [Automatic Creation of Object Hierarchies for Ray Tracing](https://typeset.io/pdf/automatic-creation-of-object-hierarchies-for-ray-tracing-9eb2zp9js9.pdf)
- [Stochastic Subsets for Bounding Volume Hierarchy (BVH) Construction](https://www.intel.cn/content/www/cn/zh/developer/articles/technical/bvh-construction.html)
