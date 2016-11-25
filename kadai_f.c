/**
 * コンパイル
 * ./make-linux.sh kadai.c -DPORT=\"/dev/tty.usbmodem14141\"
 * ただしPORTの値は必要に応じてかえる(/dev/ttyACM0?)
 */

#include <stdio.h>
#include <ctype.h>
#include <cv.h>
#include <highgui.h>
#include <curses.h>
#include <math.h>
#include <sys/time.h>
#include "./get_contour.h"
#include "./serial_rev.h"
#include "./default.h"

#define FORWARD  0
#define BACKWARD 1
#define RIGHT    2
#define LEFT     3
#define CENTER   4
#define INF     -1

void on_mouse(int event, int x, int y, int flags, void *param);
float get_distance(float coordinate);
int get_runtime(float distance);
void go_straight(int direction, float distance);
void turn(int direction, float degree);
void turn_slow(int direction);
void curve(int direction);
void round(int left, int right);
float hypote(float x, float y);
int x_average(float centers[3]);
void set_camera_v(int v);

// 現在のカメラサーボ垂直角度を格納する
int camera_angle;

int main(int argc, char **argv)
{
  CvCapture* capture = NULL;
  IplImage* frame;      // キャプチャ画像 (RGB)
  IplImage* framtPT;    // 透視変換画像 (RGB)
  IplImage* framePTHSV; // 透視変換画像 (HSV)
  IplImage* mask;       // 指定値によるmask (１チャネル)
  IplImage* contour;    // GetLargestContour() の結果
  IplImage** frames[] = {&framtPT, &framePTHSV};
  contourInfo topContoursInfo[CONTOURS];

  IplImage *frameHough;
  IplImage *frameGray = NULL;
  CvMemStorage *storage;
  CvSeq *circles = NULL;
  float *p;
  float centers[3];
  int i;

  CvMat *map_matrix;
  CvPoint2D32f src_pnt[4], dst_pnt[4];

  src_pnt[0] = cvPoint2D32f(181.0, 199.0);
  src_pnt[1] = cvPoint2D32f(110.5, 199.0);
  src_pnt[2] = cvPoint2D32f(104.7, 240.0);
  src_pnt[3] = cvPoint2D32f(184.2, 240.0);
  dst_pnt[0] = cvPoint2D32f(132.5, 240.0);
  dst_pnt[1] = cvPoint2D32f(107.5, 240.0);
  dst_pnt[2] = cvPoint2D32f(107.5, 260.0);
  dst_pnt[3] = cvPoint2D32f(132.5, 260.0);
  map_matrix = cvCreateMat (3, 3, CV_32FC1);
  cvGetPerspectiveTransform (src_pnt, dst_pnt, map_matrix);

  // RGBのチューニング配列
  uchar minH[3] = {80, 14, 42 }, maxH[3] = {140,34, 72 };
  uchar minS[3] = {100,145,76 }, maxS[3] = {220,235,149};
  uchar minV[3] = {90 ,100,58 }, maxV[3] = {255,167,127};

  if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
  if (capture == NULL) {
    printf("カメラが見つかりません\n");
    return -1;
  }

  // 解析速度向上のために画像サイズを下げる
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

  frame = cvQueryFrame(capture);
  framtPT = cvCreateImage(cvSize(240,270), IPL_DEPTH_8U, 3);
  framePTHSV = cvCreateImage(cvGetSize(framtPT), IPL_DEPTH_8U, 3);
  mask = cvCreateImage(cvGetSize(framtPT), IPL_DEPTH_8U, 1);
  contour = cvCreateImage(cvGetSize(framtPT), IPL_DEPTH_8U, 3);

  cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("contour", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("src", 60,480);
  cvMoveWindow("dst", 380,480);
  cvMoveWindow("contour", 700,480);
  cvSetMouseCallback("src", on_mouse, (void *)frames);
  cvSetMouseCallback("dst", on_mouse, (void *)frames);
  cvSetMouseCallback("contour", on_mouse, (void *)frames);

  initscr();
  init();

  // 各サーボ初期化(初期値定義はdefault.hでされている)
  motor_on(MOTOR_DEFAULT_L, MOTOR_DEFAULT_R);
  camera_on(CAMERA_CENTER_V, CAMERA_CENTER_H);
  camera_horizontal(CAMERA_INIT_H);
  set_camera_v(CAMERA_INIT_V);

  // RGBのどれをターゲットにするかのフェーズ。チューニング配列のインデックスになる
  int phase = 0;
  // systemの時間を取得して格納する構造体。<sys/time.h>
  struct timeval start, now;
  // 動作開始時刻を取得
  gettimeofday(&start,NULL);
  while (1) {
    // 現在時刻を取得
    gettimeofday(&now,NULL);
    // 低視野探索開始から何も見つけず7秒経過したら高視野に切り替える
    if (camera_angle == CAMERA_INIT_V && now.tv_sec - start.tv_sec > 7)
    {
      set_camera_v(CAMERA_UP_V);
      usleep(20000);
    }

    // カメラからの情報を取得、処理
    frame = cvQueryFrame(capture);
    cvWarpPerspective (frame, framtPT, map_matrix,
                         CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (100));
    cvCvtColor(framtPT, framePTHSV, CV_RGB2HSV);
    GetMaskHSV(framtPT, mask, minH[phase], maxH[phase], minS[phase], maxS[phase], minV[phase], maxV[phase]);
    GetLargestContour(framtPT, mask, contour, topContoursInfo);

    // 低視野か高視野かで回転速度を切り替える
    (camera_angle == CAMERA_INIT_V) ? turn(RIGHT, INF) : turn_slow(RIGHT);

    mvprintw(0,0,"not found");
    mvprintw(5,0,"time: %lu",now.tv_sec-start.tv_sec);
    mvprintw(6,0,"area: %d",(int)topContoursInfo[0].area);

  	//↑以上がカメラ角度の制御。以下モータの制御
  	
  	
    // 十分な面積が見つかったら
    if ((camera_angle == CAMERA_INIT_V && topContoursInfo[0].area > 1600.0)
                    || (camera_angle == CAMERA_UP_V && topContoursInfo[0].area > 900.0))
    {
      motor_stop();
      usleep(500000);
      frame = cvQueryFrame(capture);
      cvWarpPerspective (frame, framtPT, map_matrix,
                         CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (100));
      cvCvtColor(framtPT, framePTHSV, CV_RGB2HSV);
      GetMaskHSV(framtPT, mask, minH[phase], maxH[phase], minS[phase], maxS[phase], minV[phase], maxV[phase]);
      GetLargestContour(framtPT, mask, contour, topContoursInfo);
      float x = topContoursInfo[0].oblique.center.x;           //obliqueの中心のx座標
      float y = topContoursInfo[0].oblique.center.y;           //obliqueの中心のy座標
      float angle = fabs(topContoursInfo[0].oblique.angle);
      float rad   = angle*M_PI/180;
      float b = y + tan(angle)*x;
      float dest = float(b-tan(angle)*120);
      float sind = hypote(120-x,370-y)*sinf(rad);
      float cosd = hypote(120-x,370-y)*cosf(rad);
      CvPoint2D32f box_vtx[4];
      cvBoxPoints(topContoursInfo[0].oblique, box_vtx);     //obliqueの4点を取得
      mvprintw(0,0,"found    ");
      mvprintw(0,0,"oblique.center=(%.0f,%.0f)",x, y);
      mvprintw(1,0,"angle:%4.0f", angle);
      mvprintw(2,0,"center:(%3.0f,%3.0f)", x, y);
      int d;
      float cx[4], cy[4];

    	for(d = 0; d < 4; d++)
      {
          cx[d] = cvPointFrom32f(box_vtx[d]).x;
          cy[d] = cvPointFrom32f(box_vtx[d]).y;
          mvprintw(10+d,0,"(%3f,%3f)",cx[d], cy[d]);
      }

    	//目標(色紙の中心)が画面内にあるかつ長辺    ?????
      if(dest > 0 && dest < 270 &&
          ((hypote(cx[0]-cx[1],cy[0]-cy[1]) > hypote(cx[1]-cx[2], cy[1]-cy[2]) && x>100)
        || (hypote(cx[0]-cx[1],cy[0]-cy[1]) < hypote(cx[1]-cx[2], cy[1]-cy[2]) && x<140)))
      {
        cvCircle(framtPT, cvPoint(120, dest), 3, CV_RGB(255,0,0),1, 8, 0);
        refresh();
        cvShowImage("src", frame);
        cvShowImage("dst", framtPT);
        cvShowImage("contour", contour);
        if(hypote(cx[0]-cx[1],cy[0]-cy[1]) > hypote(cx[1]-cx[2], cy[1]-cy[2]) && x>100)    //的が左側
        {
          if(camera_angle == CAMERA_UP_V)   		//カメラ視野を高くして探してた時, 左に行ってカメラ角度をもとに戻す
          {
            turn(LEFT, 5);
            go_straight(FORWARD, get_distance(y));
            gettimeofday(&start,NULL);
            set_camera_v(CAMERA_INIT_V);
            continue;
          }
          //的に対して垂直の直線上に侵入し的のほうを向く
          turn(LEFT, 90-angle);
          go_straight(FORWARD, get_distance(sind));
          turn(RIGHT, 90);
        }
        else        //このelseは「的が右にあった時」
        {
          if(camera_angle == CAMERA_UP_V)    		//カメラ視野を高くして探してた時, 右に行ってカメラ角度を戻す 
          {
            turn(RIGHT, 5);
            /**TODO メッサ短い**/
            go_straight(FORWARD, get_distance(y));
            gettimeofday(&start,NULL);
            set_camera_v(CAMERA_INIT_V);
            continue;
          }
          //的に対して垂直の直線上に侵入し的のほうを向く
		  turn(RIGHT, angle);
          go_straight(FORWARD, get_distance(sind));
          turn(LEFT, 90);
        }

//遠過ぎたら戻るし近過ぎてももどる
        if     (get_distance(cosd)>60) go_straight(FORWARD,  get_distance(cosd)-60);
        else if(get_distance(cosd)<20) go_straight(BACKWARD, 20);
//

     	//以下hough変換による円の検出
        frameHough = cvQueryFrame(capture);
        frameGray = cvCreateImage(cvGetSize(frameHough), IPL_DEPTH_8U, 1);
        camera_vertical(80);         //カメラ角度高め
        usleep(200000);
        while(1)
        {
          frameHough = cvQueryFrame(capture);
          cvCvtColor(frameHough, frameGray, CV_RGB2GRAY);
          cvSmooth (frameGray, frameGray, CV_GAUSSIAN, 11, 0, 0, 0);
          storage = cvCreateMemStorage (0);
          circles = cvHoughCircles (frameGray, storage, CV_HOUGH_GRADIENT, 
                              1, 3.0, 20.0, 70.0, 10, MAX (frameGray->width, frameGray->height));
          for (i = 0; i < MIN(3, circles->total); i++)
          {
            p = (float *) cvGetSeqElem (circles, i);
            cvCircle (frameHough, cvPoint (cvRound (p[0]), cvRound (p[1])),
                              3, CV_RGB (0,255,0), -1, 8, 0);
            cvCircle (frameHough, cvPoint (cvRound (p[0]), cvRound (p[1])),
                              cvRound (p[2]), CV_RGB (255,0,0), 6-2*i, 8, 0);
          }
          cvShowImage("src", frameHough);
          cvShowImage("dst", frameGray);
          if(circles->total < 3) break;
          for (i = 0; i < 3; i++)
          {
            p = (float *) cvGetSeqElem (circles, i);
            centers[i] = p[0];
          }
          switch(x_average(centers))                //的の中心が平均してCASEにあったら~する
          {
            case(RIGHT):
              curve(RIGHT);
              break;
            case(LEFT):
              curve(LEFT);
              break;
            case(CENTER):
              go_straight(FORWARD, INF);
              break;
          }
        }
        /*go_straight(FORWARD, get_distance(cosd));
        go_straight(BACKWARD, 60);*/
        go_straight(FORWARD, 20);
        go_straight(BACKWARD, 40);
        if(phase==1) break;
        phase=(phase+2)%3;
        gettimeofday(&start,NULL);
        set_camera_v(CAMERA_INIT_V);
        continue;
      }
      gettimeofday(&start,NULL);
    }
    refresh();
    cvShowImage("src", frame);
    cvShowImage("dst", framtPT);
    cvShowImage("contour", contour);

    if (cvWaitKey(25) > 0) break;
  }
  finalize();
  endwin();
  cvDestroyWindow("src");
  cvDestroyWindow("dst");
  cvDestroyWindow("contour");
  cvReleaseImage(&framePTHSV);
  cvReleaseImage(&mask);
  cvReleaseImage(&contour);
  cvReleaseCapture(&capture);
  return 0;
}

//pointを置いたところが指し示す現実世界の点について画像情報を表示する
void on_mouse(int event, int x, int y, int flags, void *frames)
{
  CvScalar BGR, HSV;
  if (event == CV_EVENT_MOUSEMOVE) {
      BGR = cvGet2D(*(((IplImage***)frames)[0]), y, x);
      HSV = cvGet2D(*(((IplImage***)frames)[1]), y, x);
      mvprintw(11,0,"(%3d,%3d): RGB=(%3.0f,%3.0f,%3.0f) HSV=(%3.0f,%3.0f,%3.0f)\n",
             x, y, BGR.val[2], BGR.val[1], BGR.val[0],
             HSV.val[0], HSV.val[1], HSV.val[2]);
    }
}

/**
 * 座標から距離を得る
 * @param 座標(px)
 * @return 距離(cm)
 */
float get_distance(float coordinate)
{
    if(camera_angle == CAMERA_INIT_V) return coordinate/3.1158;
    return coordinate/2.0742;
}

/**
 * 進行時間を計算する
 * @param distance 進行距離(cm)
 * @return 進行時間(us)
 */
int get_runtime(float distance)
{
  return (int)(distance/19/*16.3*/*1000000);
}

/**
 * 回転する
 * @param direction RIGHT|LEFT
 * @param degree 回転角度、INFの場合は永久に回転する
 */
void turn(int direction, float degree)
{
  if(direction==RIGHT)
    round(20, -20);
  else
    round(-20, 20);
  if(degree!=INF)
  {
    usleep((int)(degree/63*1000000));
  }
}

void turn_slow(int direction)
{
  if(direction==RIGHT)
    round(15, -15);
  else
    round(-15, 15);
}

/**
 * 直進する
 * @param direction FORWARD|BACKWARD
 * @param distance 進行距離(cm)
 */
void go_straight(int direction, float distance)
{
  if(direction==FORWARD)
    round(20, 20);
  else
    round(-20, -20);
  if(distance!=INF)
    usleep(get_runtime(distance));
}

void curve(int direction)
{
  if(direction==RIGHT)
    round(20, 10);
  else
    round(10, 20);
}


void round(int left, int right)
{
    motor(MOTOR_DEFAULT_L+left, MOTOR_DEFAULT_R+right);
}

/**
 * 斜辺の長さ
 * @param x
 * @param y
 * @return 斜辺
 */
float hypote(float x, float y)
{
    return sqrt(x*x+y*y);
}

int x_average(float centers[3])
{
  int i;
  float ysum = 0;
  for(i = 0; i < 3; i++)
  {
    ysum += centers[i];
  }
  if(ysum / 3 < 128) return LEFT;
  if(ysum / 3 > 134) return RIGHT;
  return CENTER;
}

void set_camera_v(int v)
{
  camera_vertical(v);
  camera_angle = v;
}
