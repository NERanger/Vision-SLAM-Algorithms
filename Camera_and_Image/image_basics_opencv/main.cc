#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){

    // Read image specified by argv[1]
    cv::Mat image;
    image = cv::imread(argv[1]);

    // If the image is correctly loaded
    if(image.data == nullptr) {// Data do not exist
        cerr << "file" << argv[1] << "do not exist." << endl;
        return 0;
    }

    // Image loaded, output basic info
    cout << "image width: " << image.cols << ", height: " << image.rows << " channel number: "
    << image.channels() << endl;

    cv::imshow("image", image);
    cv::waitKey(0);  // Pause the program and wait a key input

    // Image type
    if(image.type() != CV_8UC1 && image.type() != CV_8UC3){
        // Type do not meet requirement
        cout << "Please input a color/gray image" << endl;
        return 0;
    }

    // Traverse image (the method used here can also be used for random access)
    // Use std::chrono for timer
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y < image.rows; y++){
        for(size_t x = 0; x < image.cols; x++){
            // Access the element at (x,y)
            // Use cv::Mat::ptr to get the row ptr
            unsigned char* row_ptr = image.ptr<unsigned char>(y);  // row_ptr is the head ptr of row y
            unsigned char* data_ptr = &row_ptr[x*image.channels()]; // data_ptr point to the pixel data

            // Output every channel of the pixel (1 channel if gray image)
            for(int c = 0; c != image.channels(); c++){
                unsigned char data = data_ptr[c]; // Data is the value at (x,y) and channel c
            }
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time used for traversing image: " << time_used.count() << "s" << endl;

    // Copy cv::Mat
    // Directly assignment will not copy the data
    cv::Mat image_another = image;
    // Change applied to image_another will result in the change in original image
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // Set a 100x100 rect area to 0 at top-left
    cv::imshow("image", image);
    cv::waitKey(0);

    // Use clone to copy data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;
}