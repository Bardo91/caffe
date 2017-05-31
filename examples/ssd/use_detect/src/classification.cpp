//
//
//
//
//
//

#include <iostream>
#include "WrapperSSD.h"

DEFINE_string(mean_file, "",
              "The mean file used to subtract from the input image.");
DEFINE_string(mean_value, "104,117,123",
              "If specified, can be one value or can be same as image channels"
              " - would subtract from the corresponding channel). Separated by ','."
              "Either mean_file or mean_value should be provided, not both.");
DEFINE_string(file_type, "image",
              "The file type in the list_file. Currently support image and video.");
DEFINE_string(out_file, "",
              "If provided, store the detection results in the out_file.");
DEFINE_double(confidence_threshold, 0.3,
              "Only store detections with score higher than the threshold.");


using namespace std;

int main(int _argc, char** _argv){
    const string& model_file = _argv[1];
    const string& weights_file = _argv[2];
    const string& mean_file = FLAGS_mean_file;
    const string& mean_value = FLAGS_mean_value;
    const string& file_type = FLAGS_file_type;
    const string& out_file = FLAGS_out_file;
    const float confidence_threshold = FLAGS_confidence_threshold;

    // Initialize the network.
    WrapperSSD detector(model_file, weights_file, mean_file, mean_value);

    std::string file(_argv[3]);

    cv::Mat img = cv::imread(file, -1);
    std::vector<vector<float> > detections = detector.Detect(img);

    /* Print the detection results. */
    for (int i = 0; i < detections.size(); ++i) {
        const vector<float>& d = detections[i];
        // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
        CHECK_EQ(d.size(), 7);
        const float score = d[2];
        if (score >= confidence_threshold) {
            int x0 = d[3] * img.cols;
            int y0 = d[4] * img.rows;
            int x1 = d[5] * img.cols;
            int y1 = d[6] * img.rows;




            cv::rectangle(img, cv::Rect(x0, y0, x1-x0, y1-y0),cv::Scalar(0,255,0), 3);
            cv::putText(img, std::to_string(score),cv::Point(x0, y0+10),CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
            std::string label = d[1] == 0? "plier":"wrench";
            cv::putText(img, label,cv::Point(x0, y0+20),CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
        }
    }
    cv::imshow("detection", img);
    cv::waitKey();
    return 0;
}
