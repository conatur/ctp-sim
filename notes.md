
references:

delaunays triangualtion:

Delaunay, Boris (1934). "Sur la sphère vide" [On the empty sphere]. Bulletin de l'Académie des Sciences de l'URSS, Classe des Sciences Mathématiques et Naturelles (in French). 6: 793–800.

lawsons flip algorithm:
https://www.cise.ufl.edu/~ungor/delaunay/delaunay/node5.html

simple implentation of track finding between blue and yellow in matlab:
https://blogs.mathworks.com/student-lounge/2022/10/03/path-planning-for-formula-student-driverless-cars-using-delaunay-triangulation/


what is delaunay triangles?
-> way to connect dots with triangles that give you "nice" and "well behaving" triangles rather than skinny slivers
if two triangles are not meeting the delaunay criterion, "flip" it by taking the erasing shared edge and constructing two triangles on the alternate shared edge

lawson's flip algorithm:
take any valid triangulation, look at every edge (line between ),
check if it passes the empty circle test
if the test fails, flip the edge
repeat until no edge fails

its proven that this will always terminate and you wont be stuck in a loop going back and forth flipping the same edges
n^2 flips is worst case

side note: can only flip if the quadrilateral (formed by 2 triangles) if its convex, but good for us every concave doesnt fail the test so we dont have to flip it anyways

algorithm:
1. create delaunay triangulation
2. remove exterior triangles
3. find midpoints of interior edges
4. interpolate these midpoints to create a line
5. bonus: model what the line would look like if we took a random triangulation vs delaunays to observe the difference (if any)

track.py -> generate a track, deformed circle with blue cones on inside and yellow cones on outside, with positional noise and shuffled array to simulate real data

centerline.py -> delaunay on cones, keep blue to yellow edges, order their midpoints by greedily connecting them 

path.py -> create a periodic spline over the midpoints, curvature, speed profile


Removing long edge outliers which would throw off the midpoint ->
![Cross-edge length distribution](figures/edge_lengths.png)

First cluster for straight ith blue cone to ith yellow cone
Second cluster for i+1th cone to ith cone (diagonals)
Third cluster for outliers where cones are missed causing a large edge length which will lead to the midpoints missing the truth line

Removing the third cluster outliers will reduce offset from truth but larger gaps on curved roads can make interpolation worse

-> Test error between no filter, removing, and downweighting outliters

Edge-length filtering: Cross edges above 8.0 meters are removed. On 50 generated tracks (5,055 cross edges), this removes 0.73% of midpoints. Average accuracy is unchanged: median per-track mean offset 0.0825 m -> 0.0810, p95 0.0191 m -> p95 0.0190 m. However, the right tail is significantly reduced: p90 of per-track maximum offset reduces from 0.585 m to 0.329 m, and the number of tracks containing a midpoint of an offset of 0.5 m or higher falls from 11 to 0 tracks. The filter activates on 24 out of 50 tracks and lowers the maximum on 16 of those. On those 16, median maximum falls 0.565 m -> 0.243 m. 


greedy algorithm in centerline needs a radius of 20 meters for 0 failiures over 1000 seeds