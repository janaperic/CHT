#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

using namespace cv;
using namespace std;

//function prototype for accumulator matrix calculation
Mat CalcAccumulator(Mat matrix, unsigned int r, int *p, int numw);
//function prototype for finding the global maximum in the matrix, with optional parameters
//xm and ym for the coordinates of the global maximum
int MatGlobalMax(Mat matrix);


int main(int argc, char** argv) {
  Mat img, gray, edge;
  unsigned int r_min, r_max;
  int max_el = 0;
  int prev_max_el = 0;
  int det_radius;
  //Read r_min and r_max from command line
  if(argc > 3 && argc < 5){
    r_min = atoi(argv[2]);
    r_max = atoi(argv[3]);

    if(r_min < 0 || r_max < 0) {
      cout << "Radius must have a positive value!" << endl;
      return -1;
    }

    if(r_min > r_max) {
      cout << "The minimum radius cannot be larger that the maximum one!" << endl;
      return -1;
    }

    if(r_min <= 5) {
      cout << "The minimum radius must be larger than 5 pixels" << endl;
      return -1;
    }
  }
  else{
  	cout << "Please enter a valid path to an image, the minimum and maximum radius for circle detection in pixels (for most pictures 10, 100 should suffice)" << endl;
  	cout << "Minimum radius must be larger than 5 pixels" << endl;
  	return -1;
  }
  //Read the image and store it in a matrix
  if(!(img = imread(argv[1], 1)).data) {
    cout << "Invalid file name, please enter a valid path to an image" << endl;
    cout << "Example : ./cht ../data/filename.png" << endl;
    return -1;
  }
  
  cvtColor(img, gray, COLOR_BGR2GRAY); //grayscale conversion
  cout << "Made grayscale!" << endl;
  medianBlur(gray, gray, 5); //blur the image to smooth out edges that create noise
  Canny(gray, edge, 100, 200); //Canny edge detection, store the result in matrix edge
  cout << "Made Canny Edge!" << endl;

  namedWindow("Edges", 1);
  imshow("Edges", edge);


  //extracting the size of the image
  //e.g. if the image size iz 852x480 then the image has 480 rows and 852 columns
  int height = edge.rows;
  int width = edge.cols;


  cout << "Image size is: " << width << "x" << height << " pixels" << endl;

  //Correct accumulator matrix i.e. the one with the highest maximum element
  Mat correct_acc(height, width, CV_32S);
  Mat acc(height, width, CV_32S);

  int fd;
  int numw = 0;
  int *p;
  //Device driver initialisation
  fd = open("/dev/cht", O_RDWR|O_NDELAY);
  if(fd < 0)
  {
    cout << "Cannot open driver /dev/cht" << endl;
    //return -1;
  }
  //Determining the number of white pixels
  for(int y = 0; y < height; y++) 
    for(int x = 0; x < width; x++)
      //Check if the pixel is black 
      if(edge.at<uchar>(y,x) != 0) 
        numw ++; //Number of white pixels

  int length = (numw + 1) * 4; // Number of bytes that will be allocated
  //Creating a new mapping in the virtual address space of the calling process.
  p = (int*)mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  //construct an accumulator matrix for every r in range(r_min, r_max)
  for(unsigned int r = r_min; r <= r_max; r++) {
    acc = CalcAccumulator(edge, r, p, numw);
    cout << "CALC ACCUMULATOR PASSED" << endl;

    //find the global maximum of the acc. matrix
    max_el = MatGlobalMax(acc);
    cout << "Max. element of accumulator matrix for r = " << r << " is " << max_el << endl;

    //Compares the previous max. element to the new one
    //This results in the matrix with the highest maximal value, the correct matrix
    //Other matrices are for radii that aren't contained in the image
    //Note: this approach works because we detect cirlces on dice, which have circles with the same radii
    //So one matrix contains all the centers of the circles
    if(prev_max_el < max_el) {
      correct_acc = acc;
      prev_max_el = max_el;
      det_radius = r;
    }
  }

  //Closing the driver
  close(fd);
  if(fd < 0)
  {
    cout << "Cannot close /dev/dma driver" << endl;
    //return -1;
  } 

  cout << "--------------------------------------------------------" << endl;
  cout << "BEGINNING CIRCLE DETECTION" << endl;

  int num_circles = 0; //number of detected circles
  int flag = 1; //flag variable for ending the search
  int xm;
  int ym;

  max_el = 0;
  prev_max_el = 0;

  while(flag) {
    max_el = 0;

    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
        if(max_el < correct_acc.at<int>(y,x)) {
          max_el = correct_acc.at<int>(y,x);
          xm = x;
          ym = y;
        }
      }
    }
    cout << "max. element = " << max_el << endl;
    cout << "x = " << xm << endl;
    cout << "y = " << ym << endl;
    cout << "*****************" << endl;

    //Put the 5 elements left and right of the center of the peak to 0(to eliminate them from the search)
    for(int y = ym - 5; y < ym + 6; y++) {
      for(int x = xm - 5; x < xm + 6; x++) {
        //check if one of the pixel coordinates is < 5, to avoid errors
        if(x >= 0 && y >= 0)
          correct_acc.at<int>(y,x) = 0;
      }
    }
    //if the previous peak deteriorates for 0.65, then it is not a circle, just noise
    //the value was calculated throught experiments, but might be input for the algorithm too
    //signal that the search is over by setting the flag to 0

    if(prev_max_el * 0.65 > max_el) {
      flag = 0;
    }else {
      ++num_circles;
      //Mark the circle on the image
      Point center(xm, ym);
      circle(img, center, 3, Scalar(0, 255, 0), -1, 8, 0);
      circle(img, center, det_radius, Scalar(255, 0 ,0), 3, 8, 0);
    }

    prev_max_el = max_el;

    //End the search if 6 circles are found
    if(num_circles == 6) {
      flag = 0;
      cout << "ENDED DUE TO OVERFLOW" << endl;
    }
  }

  cout << "The radius of the circles in this figure is: " << det_radius << endl;
  cout << "Number of detected circles: " << num_circles << endl;

  namedWindow("circles", 1);
  imshow("circles", img);

  waitKey(0);


  return 0;
}




int MatGlobalMax(Mat matrix) {
  int global_max = 0;

  for(int y = 0; y < matrix.rows; y++) {
    for(int x = 0; x < matrix.cols; x++) {
      if(global_max < matrix.at<int>(y,x)) {
        global_max = matrix.at<int>(y,x);
      }
    }
  }

  return global_max;
}

Mat CalcAccumulator(Mat matrix, unsigned int r, int *p, int numw) {
  //accumulator matrix, type CV_32S (i.e. int), intialized with zeros
  int height = matrix.rows;
  int width = matrix.cols;
  Mat acc = Mat::zeros(height, width, CV_32S);
  int a,b;


  cout << "HEIGHT : " << height << endl;
  cout << "WIDTH : " << width << endl; 

  int dma_seq[numw + 1];//Array of pixels that will be mapped to DMA (+1 because dma_seq[0] will hold radius value)
  int temp = 1;

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      if(matrix.at<uchar>(y,x) != 0) {
        //First 10 bits represent x location of white pixel
        //Second 10 bits represent y location of white pixel
        dma_seq[temp] = x | (y << 10);
        temp ++;
      }
    }
  }
  dma_seq[0] = r | (1 << 31);

  int length = (numw + 1) * 4;
  //With memcpy() we are copying the values of numw bytes from the location 
  //pointed to by dma_seq directly to the memory block pointed to by p.
  memcpy(p, dma_seq, length);
  //Deleting the mappings for the specified address range
  munmap(p, length);

  cout << "Finished accumulator matrix for r = " << r << endl;
  cout << "Number of white pixels = " << numw << endl;

  
  return acc;
}

