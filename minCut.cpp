#include <iostream>
#include <queue>
#include <vector>
#include <list>
#include <limits.h>
#include <float.h>
#include <math.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum connectivity // directions
{
	N = 1, NE, E, SE, S, SW, W, NW
};

struct edge
{
	Point pt;
	float weight;
};

void initialMouseCallback(int, int, int, int, void*);
void finalMouseCallback(int, int, int, int, void*);
Point neighbour(Point, int, int, int);
bool bfs(vector<vector<list<edge>>>, Point, Point, vector<vector<Point>>*, int, int, int maxCapacity = 0);
float getEdgeWeight(vector<vector<list<edge>>>, Point, Point);
bool increaseEdgeWeight(vector<vector<list<edge>>>*, Point, Point, float);
bool decreaseEdgeWeight(vector<vector<list<edge>>>*, Point, Point, float);
void markCut(vector<vector<list<edge>>>, vector<vector<list<edge>>>, Point, Mat*);

int main(int argc, char** argv)
{
	if(argc != 3)
	{
		cout << "Incorrect number of arguments" << endl;
		cout << "Usage : ./minCut <path of image> 0/1" << endl;
		cout << "0 for without capacity scaling, 1 for with capacity scaling" << endl;
		return 0; 
	}

	Mat input = imread(argv[1], IMREAD_COLOR);
	Mat gray_input;

	cvtColor(input, gray_input, COLOR_BGR2GRAY); // convert to grayscale (weighted formula)

	Mat output(gray_input.rows, gray_input.cols, gray_input.type(), Scalar(0)); // initialize same sized image - all black

	// black in output image means the pixel is not connected to any component

	namedWindow("gray", WINDOW_NORMAL); // display grayscale image
	imshow("gray", gray_input);

	waitKey(100);

	vector<Point> seeds;

	cout << "Select a point from the foreground and background respectively and then press any key" << endl;

	setMouseCallback("gray", initialMouseCallback, &seeds);

	waitKey(0);

	setMouseCallback("gray", finalMouseCallback, NULL);

	vector<vector<list<edge>>> adjList; // adjacency list
	vector<vector<Point>> parent; // 2-D parent array for recording path found using BFS 

	adjList.resize(gray_input.rows);

	parent.resize(gray_input.rows);	

	for(int i = 0; i < gray_input.rows; i++)
	{
		adjList[i].resize(gray_input.cols);
		parent[i].resize(gray_input.cols);

		fill(parent[i].begin(), parent[i].end(), Point(-1, -1)); // initialize parent array

		for(int j = 0; j < gray_input.cols; j++)
		{
			int curIntensity = gray_input.at<uchar>(i, j);

			for(int k = 1; k <= 8; k+=2) // k++ implies 8-connectivity, k+=2 implies 4 connectivity
			{
				Point nbh = neighbour(Point(j, i), k, gray_input.cols, gray_input.rows); // neighbour
				if(nbh.x != -1 && nbh.y != -1)
				{
					int adjIntensity = gray_input.at<uchar>(nbh);
					edge temp;
					temp.pt = nbh;
					temp.weight = 256-abs(curIntensity - adjIntensity); // weight of edge; higher weight implies less difference in intensities
					adjList[i][j].push_back(temp);
				}
			}
		}
	}

	vector<vector<list<edge>>> originalAdjList(adjList); // copy adjacency list

	if(atoi(argv[2]) == 0) // normal approach without capacity scaling
	{
		while(bfs(adjList, seeds[0], seeds[1], &parent, gray_input.rows, gray_input.cols)) // while there is a path from source to target (bfs funciton populates "parent")
		{
			count++;
			float flow = FLT_MAX;
			Point foo = seeds[1];
			while(foo != seeds[0]) // calculating minimum of all weights in the path; equivalent to finding minimum/bottleneck capacity in the chosen path
			{
				Point fooParent = parent[foo.y][foo.x];
				float edgeWeight = getEdgeWeight(adjList, fooParent, foo);
				if(edgeWeight < 0)
				{
					cerr << "Error!";
					return 0;
				}
				if(flow > edgeWeight)
				{
					flow = edgeWeight;
				}
				foo = fooParent;
			}

			foo = seeds[1];
			while(foo != seeds[0]) // increase and decrease edge weights by amount "flow"- minimum weight of all edges, as found above
			{
				Point fooParent = parent[foo.y][foo.x];
				increaseEdgeWeight(&adjList, foo, fooParent, flow);
				decreaseEdgeWeight(&adjList, fooParent, foo, flow);

				foo = fooParent;
			}

			for(int i = 0; i < gray_input.rows; i++) // reset parent array
			{
				fill(parent[i].begin(), parent[i].end(), Point(-1, -1));
			}
		}
	}

	else if(atoi(argv[2]) == 1) // capacity scaling approach
	{
		int maxCapacity = 256;

		while(maxCapacity >= 1)
		{
			while(bfs(adjList, seeds[0], seeds[1], &parent, gray_input.rows, gray_input.cols, maxCapacity)) // while there is a path from source to target (bfs funciton populates "parent")
			{
				count++;
				float flow = FLT_MAX;
				Point foo = seeds[1];
				while(foo != seeds[0]) // calculating minimum of all weights in the path; equivalent to finding minimum/bottleneck capacity in the chosen path
				{
					//cout << "(" << foo.y << "," << foo.x << ") ";
					Point fooParent = parent[foo.y][foo.x];
					float edgeWeight = getEdgeWeight(adjList, fooParent, foo);
					if(edgeWeight < 0)
					{
						cerr << "Error!";
						return 0;
					}
					if(flow > edgeWeight)
					{
						flow = edgeWeight;
					}
					foo = fooParent;
				}
				//cout << count << endl;

				foo = seeds[1];
				while(foo != seeds[0]) // increase and decrease edge weights by amount "flow"- minimum weight of all edges, as found above
				{
					Point fooParent = parent[foo.y][foo.x];
					increaseEdgeWeight(&adjList, foo, fooParent, flow);
					decreaseEdgeWeight(&adjList, fooParent, foo, flow);

					foo = fooParent;
				}

				for(int i = 0; i < gray_input.rows; i++) // reset parent array
				{
					fill(parent[i].begin(), parent[i].end(), Point(-1, -1));
				}
			}

			maxCapacity /= 2;
		}
	}
	else
	{
		cout << "Incorrect argument for capacity scaling" << endl;
		return 0;
	}

	markCut(originalAdjList, adjList, seeds[0], &output); // mark the cut in the output image

	namedWindow("final", WINDOW_NORMAL);
	imshow("final", output);

	waitKey(0);

	return 0;
}

void initialMouseCallback(int event, int x, int y, int flags, void* v) // record mouse clicks
{
	if(event == EVENT_LBUTTONDOWN)
	{
		cout << x << " " << y << endl;
		((vector<Point>*)v)->push_back(Point(x, y)); // enqueue
	}
}

void finalMouseCallback(int event, int x, int y, int flags, void* userdata)
{
	return;
}

Point neighbour(Point input, int direction, int cols, int rows) // calculates neighbour based on direction input
{
	switch(direction)
	{
		case N:
		{
			input.y--;
			break;
		}

		case NE:
		{
			input.x++;
			input.y--;
			break;
		}

		case E:
		{
			input.x++;
			break;
		}

		case SE:
		{
			input.x++;
			input.y++;
			break;
		}

		case S:
		{
			input.y++;
			break;
		}

		case SW:
		{
			input.x--;
			input.y++;
			break;
		}

		case W:
		{
			input.x--;
			break;
		}

		case NW:
		{
			input.x--;
			input.y--;
			break;
		}
	}
	if(input.x < 0 || input.x >= cols || input.y < 0 || input.y >= rows)
	{
		return Point(-1, -1);
	}
	return input;
}

bool bfs(vector<vector<list<edge>>> adjList, Point s, Point t, vector<vector<Point>>* parent, int rows, int cols, int maxCapacity)
{
	bool** visited = new bool*[rows]; // 2-D "visited" array
	for(int i = 0; i < rows; i++)
	{
		visited[i] = new bool[cols];
		memset(visited[i], false, sizeof(bool) * cols);
	}

	queue<Point> q;
	q.push(s);
	visited[s.y][s.x] = true;
	(*parent)[s.y][s.x] = Point(-1, -1);

	while(!q.empty() && visited[t.y][t.x] != true)
	{
		Point temp = q.front();
		q.pop();

		auto it = adjList[temp.y][temp.x].begin();
		while(it != adjList[temp.y][temp.x].end()) // iterate through neighbours of the node
		{
			if((*it).weight >= maxCapacity && !visited[(*it).pt.y][(*it).pt.x]) // if positive weight and neighbour has not been visited, enqueue, mark visited as true and mark parent
			{
				q.push((*it).pt);
				visited[(*it).pt.y][(*it).pt.x] = true;
				(*parent)[(*it).pt.y][(*it).pt.x] = temp;
			}
			it++;
		}
	}

	bool returnValue = visited[t.y][t.x];

	for(int i = 0; i < rows; i++)
	{
		delete [] visited[i];
	}

	delete [] visited;

	return returnValue;
}

float getEdgeWeight(vector<vector<list<edge>>> adjList, Point u, Point v)
{
	auto it = adjList[u.y][u.x].begin();
	while(it != adjList[u.y][u.x].end())
	{
		if((*it).pt == v)
		{
			return (*it).weight;
		}
		it++;
	}
	return -1.0;
}

bool increaseEdgeWeight(vector<vector<list<edge>>>* adjList, Point u, Point v, float increase)
{
	auto it = (*adjList)[u.y][u.x].begin();
	while(it != (*adjList)[u.y][u.x].end())
	{
		if((*it).pt == v)
		{
			(*it).weight += increase;
			return true;
		}
		it++;
	}
}

bool decreaseEdgeWeight(vector<vector<list<edge>>>* adjList, Point u, Point v, float decrease)
{
	auto it = (*adjList)[u.y][u.x].begin();
	while(it != (*adjList)[u.y][u.x].end())
	{
		if((*it).pt == v)
		{
			(*it).weight -= decrease;
			if((*it).weight < 0)
			{
				cerr << "Negative error!" << endl;
			}
			if((*it).weight < 0.00001) // assume weight is 0 and erase edge
			{
				(*adjList)[u.y][u.x].erase(it);
			}
			return true;
		}
		it++;
	}
	cerr << "Decrease error!" << endl;
	return false;
}

void markCut(vector<vector<list<edge>>> originalAdjList, vector<vector<list<edge>>> adjList, Point s, Mat* output)
{
	// initial BFS

	bool** visited = new bool*[output->rows];
	for(int i = 0; i < output->rows; i++)
	{
		visited[i] = new bool[output->cols];
		memset(visited[i], false, sizeof(bool) * output->cols);
	}

	queue<Point> q;
	q.push(s);
	visited[s.y][s.x] = true;

	while(!q.empty())
	{
		Point temp = q.front();
		q.pop();

		auto it = adjList[temp.y][temp.x].begin();
		while(it != adjList[temp.y][temp.x].end())
		{
			if((*it).weight > 0 && !visited[(*it).pt.y][(*it).pt.x])
			{
				q.push((*it).pt);
				visited[(*it).pt.y][(*it).pt.x] = true;
			}
			it++;
		}
	}

	for(int i = 0; i < output->rows; i++)
	{
		for(int j = 0; j < output->cols; j++)
		{
			if(visited[i][j]) // nodes reachable from source vertex
			{
				auto it = originalAdjList[i][j].begin();
				while(it != originalAdjList[i][j].end())
				{
					if(!visited[(*it).pt.y][(*it).pt.x]) // if it has an edge to a non-reachable vertex in the original graph, that edge is part of the min cut
					{
						output->at<uchar>(i, j) = 255;
						output->at<uchar>((*it).pt.y, (*it).pt.x) = 255;
					}
					it++;
				}
			}
		}
	}

	for(int i = 0; i < output->rows; i++)
	{
		delete [] visited[i];
	}

	delete [] visited;
}