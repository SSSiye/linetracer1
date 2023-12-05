//스마트모빌리티 2101069 이시예
//라인트레이서 영상처리 완료.

#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>        //절대값 함수를 위한 헤더파일
#include <chrono>       //시간재기위한 헤더파일
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"

using namespace cv;
using namespace std;
using namespace chrono;

bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }

/*
모의영상 이름 모음
1_lt_ccw_50rpm_out.mp4
2_lt_ccw_50rpm_in.mp4
3_lt_cw_50rpm_out.mp4
4_lt_cw_50rpm_in.mp4
5_lt_cw_100rpm_out.mp4
6_lt_ccw_100rpm_out.mp4
7_lt_ccw_100rpm_in.mp4
8_lt_cw_100rpm_in.mp4
*/

int main()
{
    VideoCapture source("7_lt_ccw_100rpm_in.mp4");
    if (!source.isOpened()) { cerr << "source empty!" << endl; return -1; }

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
        nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
        h264parse ! rtph264pay pt=96 ! \
        udpsink host=203.234.58.153 port=8881 sync=false";              //포트 8881
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer1 open failed!" << endl; return -1; }

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
       nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
       h264parse ! rtph264pay pt=96 ! \
       udpsink host=203.234.58.153 port=8882 sync=false";              //포트 8882
    VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);
    if (!writer2.isOpened()) { cerr << "Writer2 open failed!" << endl; return -1; }


    Dxl mx;
    struct timeval start, end1;
    double time1;

    signal(SIGINT, ctrlc_handler);
    if (!mx.open()) { cout << "dynamixel open error" << endl; return -1; }

    Mat frame, gray, bright, binary, color_binary;
    Mat labels, stats, centroids;
    int key = 0;
    double lastCenter = 320.0;
    int TorF = 0;
    int min = 100;
    int vel1 = 0, vel2 = 0, error = 0;
    int start_cnt = 0;

    while (key != 'q') {   //q 누르면 멈춤
        //auto start = high_resolution_clock::now();  //윈도우용 주기의 시작 시간 기록
        gettimeofday(&start, NULL);                   //리눅스용 시작 시간 기록
        source >> frame;
        if (frame.empty()) { cerr << "frame empty!" << endl; break; }
        gray = frame(Rect(0, 270, 640, 90));              //하단 1/4정도를 관심영역으로 선정

        cvtColor(gray, gray, COLOR_BGR2GRAY);             //그레이스케일 변환

        Scalar gray_mean = mean(gray);                    //gray 평균밝기 계산
        // cout << "gray mean =" << gray_mean[0] << endl;  //gray 평균밝기 출력

        bright = gray.clone();                      //bright에 그레이영상 깊은복사
        bright = bright + (100 - gray_mean[0]);     //평균밝기 100으로 보정
        //Scalar bright_mean = mean(bright);          //bright 평균밝기 계산
        // cout << "bright mean =" << bright_mean[0] << endl;  //bright 평균밝기 출력

        threshold(bright, binary, 130, 255, THRESH_BINARY);      //이진화
        // threshold(bright, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);

        cvtColor(binary, color_binary, COLOR_GRAY2BGR);          //그레이 영상을 컬러로 변환
        int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);  //레이블링

        int makeLine = 0;

        for (int i = 1; i < cnt; i++) {
            int* p = stats.ptr<int>(i);
            if (p[4] < 20) continue;
            Point center = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));  //라인 중심 좌표 저장

            if (TorF == 0) {       //초기 센터값 저장해주는 코드
                if (abs(lastCenter - center.x) < min) {      //중심(320)과 가장 가까운것 선별
                    //cout << i << "*****************X = " << center.x << "\n Y = " << center.y << endl;
                    min = abs(lastCenter - center.x);
                    lastCenter = center.x;      //중심(320)과 가까운것을 초기 센터로 저장
                    error = 320 - lastCenter;  //에러 저장
                }
                if (i == 2) TorF++;
            }
            else {
                //cout << i << "X = " << center.x << "\n Y = " << center.y << endl;
                if (abs(lastCenter - center.x) < 100) {   //전 센터값과 현재 센터값 비교해서 
                    lastCenter = center.x;                //가장 가까운 것을 전 센터(lastCenter)로 업데이트
                    makeLine = i;   //가장 가까운 라인번호 저장 (중심라인)
                    error = 320 - lastCenter;  //에러 저장
                }
            }


            if (makeLine == i) {    //중심 라인
                rectangle(color_binary, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
                circle(color_binary, center, 3, Scalar(0, 0, 255), -1);
            }

            else {                  //나머지 라인
                rectangle(color_binary, Rect(p[0], p[1], p[2], p[3]), Scalar(255), 2);
                circle(color_binary, center, 3, Scalar(255), -1);
            }
        }

        /*imshow("frame", frame);
        imshow("color_binary", color_binary);
        key = waitKey(33);
        */

        writer1 << frame;
        writer2 << color_binary;
        waitKey(10);
        if (mx.kbhit()) //키보드입력 체크
        {
            char c = mx.getch(); //키입력 받기
            switch (c)
            {
            case 's': vel1 = 100; vel2 = -100; start_cnt++; break; //전진          //s누르면 출발
            }

        }

        if (start_cnt > 0) {
            vel1 = 100 - 0.15 * error;       //에러에 따른 좌측바퀴 속도 제어
            vel2 = -(100 + 0.15 * error);    //에러에 따른 우측바퀴 속도 제어
            mx.setVelocity(vel1, vel2);
        }
        if (ctrl_c_pressed) {
            vel1 = vel2 = 0;
            mx.setVelocity(vel1, vel2);
            break; //Ctrl+c입력시 탈출
        }
        usleep(20 * 1000);
        gettimeofday(&end1, NULL);

        time1 = end1.tv_sec - start.tv_sec
            + (end1.tv_usec - start.tv_usec) / 1000000.0;
        // auto stop = high_resolution_clock::now();   //주기의 마지막 시간 기록
         //auto duration = duration_cast<milliseconds>(stop - start);   //한주기당 실행시간 구하기
         // cout << "While 한주기의 실행시간: " << duration.count() << " [msec]" << endl;    //한주기 실행시간 출력
        cout << "error: " << error << "  vel1:" << vel1 << ',' << "  vel2:" << vel2 << "  time: " << time1 << " [msec]" << endl;
    }
    return 0;
}