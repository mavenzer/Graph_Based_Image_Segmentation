#include <iostream>
#include <queue>
#include <vector>

#include <opencv2/opencv.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;

const int ADJACENCY_RANGE = 10;
const int SEED_RANGE = 50;

enum connectivity // directions
{
	N = 1, NE, E, SE, S, SW, W, NW
};

void initialMouseCallback(int, int, int, int, void*);
void finalMouseCallback(int, int, int, int, void*);
void processQueue(queue<Point>, int, int, Mat*, Mat, int, int);
Point neighbour(Point, int, int, int);
void checkAndAssign(Point, int, int, int, Vec3b, Mat*, queue<Point>*);

int main(int argc, char** argv)
{
	Mat input = imread(argv[1], IMREAD_COLOR);
	Mat gray_input;

	cvtColor(input, gray_input, COLOR_BGR2GRAY); // convert to grayscale (weighted formula)

	Mat output(gray_input.rows, gray_input.cols, gray_input.type()%7 + 16, Vec3b(0, 0, 0)); // initialize same sized image - all black

	// black in output image means the pixel is not connected to any component

	namedWindow("gray", WINDOW_NORMAL); // display grayscale image
	imshow("gray", gray_input);

	waitKey(100);

	queue<Point> seedsQueue;

	cout << "Select all seed points and then press any key" << endl;

	setMouseCallback("gray", initialMouseCallback, &seedsQueue); // record all clicked points as seed points

	waitKey(0);

	setMouseCallback("gray", finalMouseCallback, NULL); // disregard mouse input now

	int i = 1;
	int numSeeds = seedsQueue.size();

	while(!seedsQueue.empty())
	{
		queue<Point> q;
		q.push(seedsQueue.front()); // dequeue seed point
		seedsQueue.pop();

		processQueue(q, i, numSeeds, &output, gray_input, gray_input.cols, gray_input.rows); // start labelling pixels starting from seed point

		i++;
	}

	namedWindow("final", WINDOW_NORMAL);
	imshow("final", output);

	waitKey(0);

	Ptr<SURF> detector = SURF::create( 400 );


	return 0;
}

void initialMouseCallback(int event, int x, int y, int flags, void* q)
{
	if(event == EVENT_LBUTTONDOWN)
	{
		cout << x << " " << y << endl;
		((queue<Point>*)q)->push(Point(x, y)); // enqueue
	}
}

void finalMouseCallback(int event, int x, int y, int flags, void* userdata)
{
	return;
}

void processQueue(queue<Point> q, int i, int numSeeds, Mat* output, Mat input, int cols, int rows)
{
	Point seed = q.front();

	Vec3b regionIntensity;

	// each connected component is assigned a different colour calculated here
	regionIntensity[0] = (255/numSeeds)*i;
	regionIntensity[1] = ((255/numSeeds)*(i+1))%256;
	regionIntensity[2] = ((255/numSeeds)*(i-1))%256;

	output->at<Vec3b>(seed) = regionIntensity;

	int seedIntensity = input.at<uchar>(seed);

	while(!q.empty())
	{
		Point curPoint = q.front();
		q.pop();

		Point adj;
		for(int i = 1; i <= 8; i+=2) // i+=2 for 8-connectivity; i++ for 4-connectivity
		{
			adj = neighbour(curPoint, i, cols, rows);
			if(adj.x != -1 && adj.y != -1 && output->at<Vec3b>(adj)[0] == 0 && output->at<Vec3b>(adj)[1] == 0 && output->at<Vec3b>(adj)[2] == 0)
			{
				int curIntensity = input.at<uchar>(curPoint);
				int adjIntensity = input.at<uchar>(adj);

				checkAndAssign(adj, adjIntensity, curIntensity, seedIntensity, regionIntensity, output, &q);
			}
		}
	}
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

void checkAndAssign(Point adj, int adjIntensity, int curIntensity, int seedIntensity, Vec3b regionIntensity, Mat* output, queue<Point>* q)
{
	// if all intensity constraints are satisfied
	if(adjIntensity < curIntensity + ADJACENCY_RANGE && adjIntensity > curIntensity - ADJACENCY_RANGE && adjIntensity < seedIntensity + SEED_RANGE && adjIntensity > seedIntensity - SEED_RANGE)
	{
		output->at<Vec3b>(adj) = regionIntensity; // assign intensity in output image
		q->push(adj); // enqueue this point
	}
}