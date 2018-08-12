#include <iostream>
#include <list>
#include <math.h>
#include <limits.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum connectivity // directions
{
	N = 1, NE, E, SE, S, SW, W, NW
};

struct edge
{
	Point u, v;
	int weight;
};

struct node
{
	Point parent;
	int maxEdgeWeight;
	int rank;
};

Point neighbour(Point, int, int, int);
bool compareFunction(const edge& first, const edge& second);
Vec3b colourImage(Mat*, int*, int, int, int, vector<vector<node>>, vector<vector<bool>>*);

int main(int argc, char** argv)
{
	Mat input = imread(argv[1], IMREAD_COLOR);
	Mat gray_input;

	cvtColor(input, gray_input, COLOR_BGR2GRAY); // convert to grayscale (weighted formula)
	
	Mat output(gray_input.rows, gray_input.cols, gray_input.type()%7 + 16, Scalar(0, 0, 0)); // initialize same sized image - all black

	namedWindow("input", WINDOW_NORMAL); // display grayscale input
	imshow("input", gray_input);

	waitKey(0);

	int segmentCount = gray_input.rows * gray_input.cols; // number of initial segments

	list<edge> edgeList; // list of all edges

	vector<vector<node>> disjointSet; // structure to represent disjoint sets for union/find operations

	disjointSet.resize(gray_input.rows);

	for(int i = 0; i < gray_input.rows ; i++)
	{
		disjointSet[i].resize(gray_input.cols);

		for(int j = 0; j < gray_input.cols; j++)
		{
			edge temp;
			temp.u.x = j;
			temp.u.y = i;

			temp.v = neighbour(temp.u, 7, gray_input.cols, gray_input.rows); // W neighbour
			if(temp.v != Point(-1, -1))
			{
				temp.weight = abs(gray_input.at<uchar>(temp.v) - gray_input.at<uchar>(temp.u));
				edgeList.push_back(temp);
			}

			temp.v = neighbour(temp.u, 8, gray_input.cols, gray_input.rows); // NW neighbour
			if(temp.v != Point(-1, -1))
			{
				temp.weight = abs(gray_input.at<uchar>(temp.v) - gray_input.at<uchar>(temp.u));
				edgeList.push_back(temp);
			}

			temp.v = neighbour(temp.u, 1, gray_input.cols, gray_input.rows); // N neighbour
			if(temp.v != Point(-1, -1))
			{
				temp.weight = abs(gray_input.at<uchar>(temp.v) - gray_input.at<uchar>(temp.u));
				edgeList.push_back(temp);
			}

			temp.v = neighbour(temp.u, 2, gray_input.cols, gray_input.rows); // NE neighbour
			if(temp.v != Point(-1, -1))
			{
				temp.weight = abs(gray_input.at<uchar>(temp.v) - gray_input.at<uchar>(temp.u));
				edgeList.push_back(temp);
			}

			disjointSet[i][j].parent.x = j;
			disjointSet[i][j].parent.y = i;
			disjointSet[i][j].maxEdgeWeight = INT_MAX; // denotes independent segment
			disjointSet[i][j].rank = 1;
		}
	}

	edgeList.sort(compareFunction); // sort in ascending order according to weights

	auto it = edgeList.begin();
	while(it != edgeList.end()) // iterate through sorted edges
	{
		edge temp = *it;

		Point uParent = temp.u;
		Point vParent = temp.v;

		while(disjointSet[uParent.y][uParent.x].parent != uParent) // find parent of "u"
		{
			uParent = disjointSet[uParent.y][uParent.x].parent;
		}
		while(disjointSet[vParent.y][vParent.x].parent != vParent) // find parent of "v"
		{
			vParent = disjointSet[vParent.y][vParent.x].parent;
		}

		if(uParent != vParent) // if they are not in the same segment
		{
			// if the edge weight is lesser than max weight of either segments (comparing edge weight with "maxEdgeWeight" of parent of both segments)
			if(temp.weight < disjointSet[uParent.y][uParent.x].maxEdgeWeight || temp.weight < disjointSet[vParent.y][vParent.x].maxEdgeWeight)
			{
				// calculation of maximum weight, ignoring INT_MAX 
				int newMaxWeight = disjointSet[temp.u.y][temp.u.x].maxEdgeWeight;
				if(newMaxWeight == INT_MAX)
				{
					newMaxWeight = disjointSet[temp.v.y][temp.v.x].maxEdgeWeight;
				}

				if(newMaxWeight < disjointSet[temp.v.y][temp.v.x].maxEdgeWeight && disjointSet[temp.v.y][temp.v.x].maxEdgeWeight != INT_MAX)
				{
					newMaxWeight = disjointSet[temp.v.y][temp.v.x].maxEdgeWeight;
				}

				if(disjointSet[temp.u.y][temp.u.x].maxEdgeWeight == INT_MAX && disjointSet[temp.v.y][temp.v.x].maxEdgeWeight == INT_MAX)
				{
					newMaxWeight = temp.weight;
				}

				// update maximum weight in all parents till root --> !! Does NOT update "maxEdgeWeight" in all pixels belonging to that segment !!

				uParent = temp.u;
				vParent = temp.v;

				disjointSet[uParent.y][uParent.x].maxEdgeWeight = newMaxWeight;
				while(disjointSet[uParent.y][uParent.x].parent != uParent)
				{
					uParent = disjointSet[uParent.y][uParent.x].parent;
					disjointSet[uParent.y][uParent.x].maxEdgeWeight = newMaxWeight;
				}

				disjointSet[vParent.y][vParent.x].maxEdgeWeight = newMaxWeight;
				while(disjointSet[vParent.y][vParent.x].parent != vParent)
				{
					vParent = disjointSet[vParent.y][vParent.x].parent;
					disjointSet[vParent.y][vParent.x].maxEdgeWeight = newMaxWeight;
				}

				// do union

				if(disjointSet[uParent.y][uParent.x].rank < disjointSet[vParent.y][vParent.x].rank)
				{
					disjointSet[uParent.y][uParent.x].parent = vParent;
				}
				else if(disjointSet[vParent.y][vParent.x].rank < disjointSet[uParent.y][uParent.x].rank)
				{
					disjointSet[vParent.y][vParent.x].parent = uParent;
				}
				else
				{
					disjointSet[uParent.y][uParent.x].rank++;
					disjointSet[vParent.y][vParent.x].parent = uParent;
				}

				segmentCount--; // update count
			}	
		}
		
		it++;
	}

	vector<vector<bool>> visited; // 2D array to keep track of which nodes have been coloured
	visited.resize(gray_input.rows);

	int colourCount = 0;

	for(int i = 0; i < gray_input.rows; i++)
	{
		visited[i].resize(gray_input.cols);
		fill(visited[i].begin(), visited[i].end(), false);
	}

	for(int i = 0; i < gray_input.rows; i++)
	{
		for(int j = 0; j < gray_input.cols; j++)
		{
			if(!visited[i][j])
			{
				colourImage(&output, &colourCount, segmentCount, i, j, disjointSet, &visited);
			}
		}
	}

	namedWindow("final", WINDOW_NORMAL); // display output image
	imshow("final", output);

	waitKey(0);

	return 0;
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

bool compareFunction(const edge& first, const edge& second) // compare function for sorting list of edges according to edge weights
{
	return (first.weight < second.weight);
}

Vec3b colourImage(Mat* output, int* colourCount, int segmentCount, int i, int j, vector<vector<node>> disjointSet, vector<vector<bool>>* visited) // colours pixels
{
	if(disjointSet[i][j].parent == Point(j, i)) // if pixel is in an independent segment or is the root of a segment
	{
		if(output->at<Vec3b>(i, j) != Vec3b(0, 0, 0)) // if it has been coloured
		{
			(*visited)[i][j] = true;

			return output->at<Vec3b>(i, j);
		}

		// if it has not been coloured, calculate new colour

		(*colourCount)++;

		Vec3b regionIntensity;

		// each connected component is assigned a different colour calculated here
		regionIntensity[0] = (255/segmentCount)*(*colourCount);
		regionIntensity[1] = ((255/segmentCount)*((*colourCount)+1))%256;
		regionIntensity[2] = ((255/segmentCount)*((*colourCount)-1))%256;

		output->at<Vec3b>(i, j) = regionIntensity;

		(*visited)[i][j] = true;

		return regionIntensity;
	}

	// colour parent pixel recursively and colour this pixel with the same colour as parent

	Vec3b regionIntensity = colourImage(output, colourCount, segmentCount, disjointSet[i][j].parent.y, disjointSet[i][j].parent.x, disjointSet, visited);

	output->at<Vec3b>(i, j) = regionIntensity;

	(*visited)[i][j] = true;

	return regionIntensity;
}