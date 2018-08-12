# Graph Based Image Segmentation
Graph Based image segmentation

1. Run "cmake ."  to compile programs using cmake (sample CMakeLists.txt file is included)
2. Run "make"
3. Execute program with first argument as (relative) image path. A second argument (0 or 1) is required for the "minCut" program. 0 denotes execution without capacity scaling approach and 1 denotes execution with capacity scaling approach.

Examples:

./ccl test.jpg
./minCut test.jpg 0
./minCut test.jpg 1
./mst test.jpg
