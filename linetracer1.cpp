//����Ʈ�����Ƽ 2101069 �̽ÿ�
//����Ʈ���̼� ����ó�� �Ϸ�.

#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>        //���밪 �Լ��� ���� �������
#include <chrono>       //�ð�������� �������
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
���ǿ��� �̸� ����
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
        udpsink host=203.234.58.153 port=8881 sync=false";              //��Ʈ 8881
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer1 open failed!" << endl; return -1; }

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
       nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
       h264parse ! rtph264pay pt=96 ! \
       udpsink host=203.234.58.153 port=8882 sync=false";              //��Ʈ 8882
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

    while (key != 'q') {   //q ������ ����
        //auto start = high_resolution_clock::now();  //������� �ֱ��� ���� �ð� ���
        gettimeofday(&start, NULL);                   //�������� ���� �ð� ���
        source >> frame;
        if (frame.empty()) { cerr << "frame empty!" << endl; break; }
        gray = frame(Rect(0, 270, 640, 90));              //�ϴ� 1/4������ ���ɿ������� ����

        cvtColor(gray, gray, COLOR_BGR2GRAY);             //�׷��̽����� ��ȯ

        Scalar gray_mean = mean(gray);                    //gray ��չ�� ���
        // cout << "gray mean =" << gray_mean[0] << endl;  //gray ��չ�� ���

        bright = gray.clone();                      //bright�� �׷��̿��� ��������
        bright = bright + (100 - gray_mean[0]);     //��չ�� 100���� ����
        //Scalar bright_mean = mean(bright);          //bright ��չ�� ���
        // cout << "bright mean =" << bright_mean[0] << endl;  //bright ��չ�� ���

        threshold(bright, binary, 130, 255, THRESH_BINARY);      //����ȭ
        // threshold(bright, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);

        cvtColor(binary, color_binary, COLOR_GRAY2BGR);          //�׷��� ������ �÷��� ��ȯ
        int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);  //���̺�

        int makeLine = 0;

        for (int i = 1; i < cnt; i++) {
            int* p = stats.ptr<int>(i);
            if (p[4] < 20) continue;
            Point center = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));  //���� �߽� ��ǥ ����

            if (TorF == 0) {       //�ʱ� ���Ͱ� �������ִ� �ڵ�
                if (abs(lastCenter - center.x) < min) {      //�߽�(320)�� ���� ������ ����
                    //cout << i << "*****************X = " << center.x << "\n Y = " << center.y << endl;
                    min = abs(lastCenter - center.x);
                    lastCenter = center.x;      //�߽�(320)�� �������� �ʱ� ���ͷ� ����
                    error = 320 - lastCenter;  //���� ����
                }
                if (i == 2) TorF++;
            }
            else {
                //cout << i << "X = " << center.x << "\n Y = " << center.y << endl;
                if (abs(lastCenter - center.x) < 100) {   //�� ���Ͱ��� ���� ���Ͱ� ���ؼ� 
                    lastCenter = center.x;                //���� ����� ���� �� ����(lastCenter)�� ������Ʈ
                    makeLine = i;   //���� ����� ���ι�ȣ ���� (�߽ɶ���)
                    error = 320 - lastCenter;  //���� ����
                }
            }


            if (makeLine == i) {    //�߽� ����
                rectangle(color_binary, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
                circle(color_binary, center, 3, Scalar(0, 0, 255), -1);
            }

            else {                  //������ ����
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
        if (mx.kbhit()) //Ű�����Է� üũ
        {
            char c = mx.getch(); //Ű�Է� �ޱ�
            switch (c)
            {
            case 's': vel1 = 100; vel2 = -100; start_cnt++; break; //����          //s������ ���
            }

        }

        if (start_cnt > 0) {
            vel1 = 100 - 0.15 * error;       //������ ���� �������� �ӵ� ����
            vel2 = -(100 + 0.15 * error);    //������ ���� �������� �ӵ� ����
            mx.setVelocity(vel1, vel2);
        }
        if (ctrl_c_pressed) {
            vel1 = vel2 = 0;
            mx.setVelocity(vel1, vel2);
            break; //Ctrl+c�Է½� Ż��
        }
        usleep(20 * 1000);
        gettimeofday(&end1, NULL);

        time1 = end1.tv_sec - start.tv_sec
            + (end1.tv_usec - start.tv_usec) / 1000000.0;
        // auto stop = high_resolution_clock::now();   //�ֱ��� ������ �ð� ���
         //auto duration = duration_cast<milliseconds>(stop - start);   //���ֱ�� ����ð� ���ϱ�
         // cout << "While ���ֱ��� ����ð�: " << duration.count() << " [msec]" << endl;    //���ֱ� ����ð� ���
        cout << "error: " << error << "  vel1:" << vel1 << ',' << "  vel2:" << vel2 << "  time: " << time1 << " [msec]" << endl;
    }
    return 0;
}