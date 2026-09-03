#include "marker_detector.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
int main(int argc,char**argv){ if(argc<2){std::cerr<<"dataset path required\n";return 2;} maixcam_marker::DetectorConfig cfg{}; maixcam_marker::MarkerDetector detector(cfg); int n=0; for(auto&e:std::filesystem::directory_iterator(argv[1])) if(e.path().extension()==".pgm"){auto im=cv::imread(e.path().string(),cv::IMREAD_GRAYSCALE); if(im.empty()||im.cols!=640||im.rows!=360)return 3; auto result=detector.process(im,std::chrono::steady_clock::now()); (void)result; ++n;} detector.reset(); std::cout<<"processed "<<n<<" fixtures\n"; return n?0:4; }
