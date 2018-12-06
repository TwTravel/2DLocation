g++ -std=c++11  -I ./include HandEyeCalib.cpp  main.cpp -o main -L ./lib -lvisp_vs -lvisp_core  -L /data/tzwang/opencv_dev/opencv_contrib-4.0.0/build/lib -lopencv_core -lopencv_aruco -lopencv_videoio -lopencv_highgui -lopencv_imgcodecs  -lvisp_vision

