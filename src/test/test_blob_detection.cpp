#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main() {
    // Read the input image

    // Create SimpleBlobDetector parameters
    cv::SimpleBlobDetector::Params params;
    // // params.minThreshold = 0;
    // // params.maxThreshold = 255;
    // params.filterByColor = true;
    // params.blobColor = 255;  // Blobs should be white
    // // params.filterByArea = true;
    // // params.minArea = 50;
    // // params.maxArea = 16000;
    // // params.filterByCircularity = true;
    // // params.minCircularity = 0.0;
    // params.filterByConvexity = true;
    // params.minConvexity = 0.1;
    // // params.filterByInertia = true;
    // // params.minInertiaRatio = 0.01;

    params.filterByColor = true;
    params.blobColor = 255;  // Blobs should be white
    params.filterByArea = true;
    params.minArea = 10;
    params.maxArea = 6000;
    params.filterByCircularity = false;
    params.minCircularity = 1;
    params.filterByConvexity = true;
    params.minConvexity = 0.1;

    // Create a SimpleBlobDetector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // // Detect blobs
    // std::vector<cv::KeyPoint> keypoints;
    // detector->detect(image, keypoints);

    // // Draw detected blobs on the image
    // cv::Mat outputImage;
    // cv::drawKeypoints(image, keypoints, outputImage, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    // cv::resize(outputImage, outputImage, cv::Size(), 4.0, 4.0);

    // // Display the output image
    // cv::imshow("Blobs Detected", outputImage);
    // cv::waitKey(0);



	std::string folderPath = "/home/zhefan/Desktop/map"; // Change this to your image folder path

    // Get a list of image file paths in the folder
    std::vector<cv::String> imagePaths;
    cv::glob(folderPath + "/*.jpg", imagePaths);

    // Iterate through the image file paths
    for (const cv::String &imagePath : imagePaths) {
        // Read the image
        cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

        if (image.empty()) {
            std::cerr << "Error: Could not read image " << imagePath << std::endl;
            continue;
        }

        // Convert the image to grayscale
        cv::Mat grayImage;
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

        int x = 0;
        int y = 0;
        int width = image.size().width;
        int height = image.size().height;
        cv::Rect rect(x, y, width, height);
        cv::rectangle(image, rect, cv::Scalar(0, 0, 0), 3);
        
	    // Detect blobs
	    std::vector<cv::KeyPoint> keypoints;
	    detector->detect(image, keypoints);

	    // Draw detected blobs on the image
	    cv::Mat outputImage;
	    cv::drawKeypoints(image, keypoints, outputImage, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


	    cv::resize(outputImage, outputImage, cv::Size(), 4.0, 4.0);

	    // Display the output image
	    cv::imshow("Blobs Detected", outputImage);
	    cv::waitKey(0);

        // Display the image with detected blobs
        // cv::imshow("Blobs Detected", image);

        // Wait for a key press and close the window
        // cv::waitKey(0);
        cv::destroyAllWindows();
    }


    return 0;
}