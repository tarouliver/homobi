#include <stdio.h>
#include <ctype.h>
#include <cv.h>
#include <highgui.h>
#include <curses.h>
#include "get_contour.h"
#include <math.h>

#define PORT "/dev/ttyACM0" //適宜変更のこと
#include "serial2016.h"

#define CAMERA_CENTER_H 90 //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define CAMERA_CENTER_V 90 //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define MOTOR_DEFAULT_L 128 //左モータのデフォルト値（キャリブレーションに利用）
#define MOTOR_DEFAULT_R 128 //右モータのデフォルト値（キャリブレーションに利用）
#define CAMERA_INIT_V 70 //カメラサーボの垂直方向初期値
#define CAMERA_INIT_H 90 //カメラサーボの水平方向初期値


void on_mouse(int event, int x, int y, int flags, void *param);
int RIGHT= 0;
int LEFT = 1;
float TIME_OF_H = 10.2;

float get_xxyy(float x, float y){
  return sqrt(x*x+y*y);
}

float get_distance(float dt){
    //dt = get_xxyy(x, y);
    return dt/2.0742;
}

float get_radian(float degree){
  return degree*0.0174532925;
}

void turn(int direction, float angle){
	float timeFor90 = 2442000;
    float timeForangle = angle*timeFor90/90;
  if(direction == RIGHT){
    motor(140, 116);//右回転
    usleep(timeForangle);
    motor(128, 128);//stop
    sleep(1);
  }
  if(direction == LEFT){
    motor(116, 140);//左回転
    usleep(timeForangle);
    motor(128, 128);//stop
    sleep(1);
  } 
}

//指定された距離だけ直進する関数
void move(float distance){
	//1s = 6.5cm = 6.5*2.3px
	float time = distance/6.5/2.3;
	printf("move %fseconds\n", time);
	motor(140, 140);
	sleep(time);
	
}

int main(int argc, char **argv)
{
  CvCapture *capture = NULL;
  IplImage *frame;     // キャプチャ画像 (RGB)
  IplImage* frameHSV;  // キャプチャ画像 (HSV)
  IplImage* mask;      // 指定値によるmask (１チャネル)
  IplImage* contour;   // GetLargestContour() の結果
  IplImage** frames[] = {&frame, &frameHSV};
  contourInfo topContoursInfo[CONTOURS];
  int key;
  
  //直進カウンタ
  int stCount=0;
  //回転フラグ(1右回転　2左回転)
  int rotaFlag=1;


  init();

  motor_on(121, 130); // モーター静止パルス幅のキャリブレーション
  camera_on(94, 92);    // カメラアングルキャリブレーション

  camera_horizontal(CAMERA_INIT_H); // 水平方向のカメラ角度を初期値に
  camera_vertical(CAMERA_INIT_V); // 垂直方向のカメラ角度を初期値に

  // 赤系のHSV色．各自チューニングすること
  uchar minH = 110, maxH = 140;
  uchar minS = 100, maxS = 255;
  uchar minV =  60, maxV = 255;

  if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
  if (capture == NULL) {
    printf("not find camera\n");
    return -1;
  }

  // 解析速度向上のために画像サイズを下げる
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

  frame = cvQueryFrame(capture);
  frameHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
  mask = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  contour = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

  cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("contour", CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback("src", on_mouse, (void *)frames);
  cvSetMouseCallback("contour", on_mouse, (void *)frames);
  
  float theangle;
  float theradian;
  float thedistance;
  
  while (1) {
    frame = cvQueryFrame(capture);
    cvCvtColor(frame, frameHSV, CV_RGB2HSV);
    cvShowImage("src", frame);
    GetMaskHSV(frame, mask, minH, maxH, minS, maxS, minV, maxV);
    GetLargestContour(frame, mask, contour, topContoursInfo);
    cvShowImage("contour", contour);
    key = cvWaitKey(1);
	//見つけるまでは回転する
    if(topContoursInfo[0].area > 0){
      /// 赤い物体を見つけた場合
      CvBox2D oblique = topContoursInfo[0].oblique; // 認識した物体を囲む長方形
      float x = oblique.center.x;                            // 認識した物体の画面内のx座標(0~319)
      float y = oblique.center.y;                            // 認識した物体の画面内のy座標(0~239)
      float angle = fabs(oblique.angle); 
      float radian = get_radian(angle);
      float distance = get_distance(get_xxyy(x, y));	//距離（pix）
	printf("x=%f, y=%f,  angle=%f radian=%f \n", x, y, angle, radian);
	printf("distance=%f\n", distance);
	/*
	angleの値を読み続けるのはカメラを向け続ける必要があり難しい。
	100確定後カメラが止まるので注意。治すべき点。
	手順は、
	（1. 最初は回転）
	2. まず十分な大きさまで近づく。
	3. 四点の関係から、左か右の垂線かを判断する。
	4. その場で回転。距離（rcos(rad)）だけ動く。
	*/
	//まず十分な大きさまで近づく。
    if(150< x && x <170){
    	motor(140, 140);
      if(get_distance(get_xxyy(x, y)) > 100){//100=1m?
        //ここで回転方向決定したい
        theangle = angle;
        theradian = radian;
        thedistance = distance;
        printf("the angle =%f, the radian = %f, the distance=%f\n", theangle, theradian, thedistance);
        turn(LEFT, theangle);
        move(thedistance*cos(theradian));
        turn(RIGHT, 90);
        move(thedistance*sin(theradian));
      }
    }else if(x <= 150){
      		motor(133 ,140);
        }else{
		motor(140, 133);
	}
    }else{
      /// 赤い物体が見つからない場合その場回転する
      motor(135, 121);
    }
    if (key == 'q') break;
}

  //以下いじらない

  finalize();
  cvDestroyWindow("src");
  cvDestroyWindow("contour");
  cvReleaseImage(&frameHSV);
  cvReleaseImage(&mask);
  cvReleaseImage(&contour);
  cvReleaseCapture(&capture);
  return 0;
}

void on_mouse(int event, int x, int y, int flags, void *frames)
{
  CvScalar BGR, HSV;
  if (event == CV_EVENT_MOUSEMOVE) {
      BGR = cvGet2D(*(((IplImage***)frames)[0]), y, x);
      HSV = cvGet2D(*(((IplImage***)frames)[1]), y, x);
      printf("(%3d,%3d): RGB=(%3.0f,%3.0f,%3.0f) HSV=(%3.0f,%3.0f,%3.0f)\n",
             x, y, BGR.val[2], BGR.val[1], BGR.val[0],
             HSV.val[0], HSV.val[1], HSV.val[2]);
    }
}
