all: lane_detect.o CKalmanFilter.o main.cpp
	g++ -std=c++0x -g lane_detect.o CKalmanFilter.o main.cpp -o main `pkg-config --cflags --libs opencv`

lane_detect.o: laneDetection.cpp laneDetection.h
	g++ -std=c++0x -c laneDetection.cpp -o lane_detect.o

lane_detect: CKalmanFilter.o laneDetection.cpp laneDetection.h
	g++ -std=c++0x -D LaneTest CKalmanFilter.o laneDetection.cpp -o lane_detect `pkg-config --cflags --libs opencv`

CKalmanFilter.o: CKalmanFilter.cpp CKalmanFilter.h
	g++ -std=c++0x -c CKalmanFilter.cpp -o CKalmanFilter.o

clean:
	rm -f main lane_detect *.o 