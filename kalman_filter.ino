#include <M5Stack.h>
#include <math.h>
#include "utility/MPU9250.h"

MPU9250 IMU;

int col_top = TFT_BLACK, col_bottom = TFT_GREEN;

const float hx = 160, hy = 120;
int qx = hx, qy = 0;

int count = 0;
int function_state= 0;

float acc_x = 0;
float acc_y = 0;
float gyro_z = 0;

float gyro_z_sum = 0;
float gyro_z_offset = 0;

float time_n = 0;
float time_p = 0;
float dt = 0;

float theta = 0;
float theta_acc = 0;
float theta_flitered = 0;
float theta_forecast = 0;
float theta_previous = 0;

float sigma_theta_forecast = 0;
float sigma_theta_previous = 0;
float sigma_theta_filtered = 0;
float sigma_theta_eq = 0;
float sigma_theta_obs = 0;
float kGain = 1;
float diff_theta = 0;

const float d = atan2(hy, hx);

#define PI 3.14159265358979323846


template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

void triangle(int x1, int y1, int x2, int y2, int x3, int y3, int color)
{
  M5.Lcd.fillTriangle(hx + x1, hy + y1, hx + x2, hy + y2, hx + x3, hy + y3, color);
}

void setup()
{
  M5.begin();
  Wire.begin();

  IMU.initMPU9250();
  IMU.getAres();
  IMU.getGres();
  M5.Lcd.fillRect(0, hy, 2 * hx, 2 * hy, col_bottom);
}

void loop()
{
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    
    IMU.readAccelData(IMU.accelCount);
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes;
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes;
    acc_x = IMU.ax;
    acc_y = -IMU.ay;
    theta_acc = atan2(acc_x, acc_y);

    IMU.readGyroData(IMU.gyroCount);
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
    gyro_z = -IMU.gz/180*PI;

    time_n = millis()/(float)1000;

    dt = time_n - time_p;
    
    M5.update();

    //ジャイロのキャリブレーション
    if (count < 100 && function_state == 0)
    {
      if (count == 0)
      {
        gyro_z_sum = 0;
      }
      gyro_z_sum = gyro_z_sum + gyro_z;
      if (count == 99)
      {
        gyro_z_offset = gyro_z_sum/100;
        function_state = 1;
      }
    }
    //左のボタンを押したとき
    else if (M5.BtnA.wasPressed()==1)
    {
      function_state = 1;
      col_bottom = TFT_GREEN;
    }
    //中央のボタンを押したとき
    else if (M5.BtnB.wasPressed()==1)
    {
      function_state = 2;
      count = 0;
      col_bottom = TFT_YELLOW;
    }
    //右のボタンを押したとき
    else if (M5.BtnC.wasPressed()==1)
    {
      function_state = 3;      
      count = 0;
      col_bottom = TFT_BLUE;
    }

    //加速度のみ
    if (function_state == 1)
    {
      theta = theta_acc;
    }

    //ジャイロのみ
    else if (function_state == 2)
    {
      if (count == 0)
      {
        theta = theta_acc;
      }
      else
      {
        theta = theta + (gyro_z-gyro_z_offset)*dt;
      }
    }

    //カルマンフィルタ
    else if  (function_state == 3)
    {
      
        //初期値代入
      if (count == 0)
      {
        sigma_theta_eq = (float)1/180*PI;
        sigma_theta_obs = (float)50/180*PI;      
        theta_previous = theta_acc; 
        sigma_theta_previous = sigma_theta_obs;
        theta = theta_previous;
      }
      else
      {
        //予測ステップ
        theta_forecast = theta_previous + (gyro_z - gyro_z_offset)*dt;
        if (theta_forecast >PI)
        {
          theta_forecast = theta_forecast - 2*PI;
        }
        else if (theta_forecast<=-PI)
        {
          theta_forecast = theta_forecast + 2*PI;
        }
        sigma_theta_forecast = sigma_theta_previous + sigma_theta_eq;
    
        //フィルタリングステップ
        kGain = sigma_theta_forecast/(sigma_theta_forecast+sigma_theta_obs);
        
        diff_theta = theta_acc-theta_forecast;
        if (diff_theta >= PI)
        {
          diff_theta = diff_theta - 2*PI;
        }
        else if (diff_theta < -PI)
        {
          diff_theta = 2*PI + diff_theta;      
        }
        
        theta_flitered = theta_forecast + kGain*(diff_theta);
        sigma_theta_filtered = (1-kGain)*sigma_theta_forecast;

        theta = theta_flitered;
    
        theta_previous = theta_flitered;
        sigma_theta_previous = sigma_theta_filtered;
      }
    }

    //描画処理
    int px, py;

    if (fabs(hx * tan(-theta)) < hy)
    {
      px = hx * sgn(cos(-theta));
      py = px * tan(-theta);
    }
    else
    {
      py = hy * sgn(sin(-theta));
      px = py / tan(-theta);
    }

    int col1, col2;
    if (px * qy - py * qx > 0)
    {
      col1 = col_bottom;
      col2 = col_top;
    }
    else
    {
      col1 = col_top;
      col2 = col_bottom;
    }

    triangle(0, 0, qx, qy, px, py, col2);
    triangle(0, 0, -qx, -qy, -px, -py, col1);

    if (qx != px && qy != py)
    {
      int cx = hx * sgn(qx);
      int cy = hy * sgn(qy);

      triangle(cx, cy, qx, qy, px, py, col2);
      triangle(-cx, -cy, -qx, -qy, -px, -py, col1);
    }

    qx = px;
    qy = py;
    time_p = time_n;
    count = count + 1;
  }
}
