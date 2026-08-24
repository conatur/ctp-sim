
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
