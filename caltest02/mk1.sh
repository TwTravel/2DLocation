g++ -g  -std=c++11 detect_markers.cpp HandEyeCalib.cpp -I ./include  -o decmarker -L /data/tzwang/opencv_dev/opencv_contrib-4.0.0/build/lib -lopencv_core -lopencv_aruco -lopencv_videoio -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -L ./lib -lvisp_vs -lvisp_core  -lvisp_vision -fpermissive

