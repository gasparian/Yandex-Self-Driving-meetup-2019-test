## Road plane detection

To get the invitation to the private meetup in Yandex self-driving, the participants had to solve a simple computer-vision problem.  
(I've passed).

### Task  

Detect the plane of the road in static lidar points cloud. The plane is described by four coefficients.  
The solution was evaluated for accuracy and performance.  
Limits: 64 mb RAM; 15 sec. running on single core of CPU.  
<img src="https://github.com/gasparian/Yandex_self-driving_meetup-2019_test/blob/master/imgs/road.png" height=500>  
There were some test examples given, which contains points coordinates in 3D space.  
Also we knew that the road takes more than 50% of the points and these points should be concetrated in some neighborhood determined by input value `p`.  

### Solution  

We can fit a simple linear regression on given points coordinates. But there are a lot of outliers in the data, which will cause the problems in detecting road plane. So it's better to use the [ransac regression](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm in this case, for its' robustness.  
The main idea is to iteratively sample triplets of points, find a plane and calculate the number of neighbors. After finding the good initial plane, we can now fit a plane using all neighbors found.   
But for sake of speed-up, we need to somehow stop iteratining earlier. We can do it in varouous ways: empiracally find optimal max. amount of iterations, which provides minimum value of plane coefficients dispersion; stop iterating after finding the plane wich has more than a 50% of points being its' neighbors and so on.  
In my implementation I used both methods, but keep in mind - it's less guaranteed to find the accurate plane using the second approach, but on the other hand we may accidentally get a "good" set of points in the early iterations and almost instantly solve a problem.  
So I had the folowing results:  
 - 2.5 sec. & 5.12 Mb - "baseline";  
 - 46ms & 1.46 mb - the best result passing all the tests.  

### Running  

I've been using `GNU C++11 4.9` and the folowing compilation settings:  
`g++ -O2 -fno-stack-limit -x c++ detect_road_plane.cpp -o detect_road_plane`  
You can also use `performance_measure.sh` for speed measures.  
