/*
 *  semaseg_camera.ino - Binary Sematic Segmentation sample
 *  Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Camera.h>
#include "Adafruit_ILI9341.h"
#include <DNNRT.h>
#include <Flash.h>
#include <SDHCI.h>

//#define EXT_BOARD
//#define LTE_BOARD
//#define MAIN_BOARD
#define MAIN_BOARD_WITH_LTE

#ifdef EXT_BOARD
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10
#define SPI_CLOCK 40000000
const int intPin = 4;
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
#endif

#ifdef LTE_BOARD
#define TFT_DC  6
#define TFT_CS  -1
#define TFT_RST 2
#define SPI_CLOCK 6500000
const int intPin = 9;
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI3, TFT_DC, TFT_CS, TFT_RST);
#endif

#ifdef MAIN_BOARD
#define TFT_RST 18
#define TFT_DC  25
#define TFT_CS  -1
const int intPin = 0;
#define SPI_CLOCK 20000000
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI5, TFT_DC, TFT_CS, TFT_RST);
#endif

#ifdef MAIN_BOARD_WITH_LTE
#define TFT_DC  6
#define TFT_CS  -1
#define TFT_RST 2
const int intPin = 9;
#define SPI_CLOCK 20000000
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI5, TFT_DC, TFT_CS, TFT_RST);
#endif


#define OFFSET_X  (32)
#define OFFSET_Y  (24)
#define CLIP_WIDTH  (256)
#define CLIP_HEIGHT  (192)
#define DNN_WIDTH  (64)
#define DNN_HEIGHT  (48)
#define DNN2_WIDTH (58)
#define DNN2_HEIGHT (36)
#define DNN3_WIDTH (18)
#define DNN3_HEIGHT (30)

// #define USE_SD_CARD
// #define SAVE_NUM_IMAGE

#ifdef USE_SD_CARD
#include "BmpImage.h"
BmpImage bmp;
SDClass SD;
#endif

DNNRT dnnrt;
// RGBの画像を入力
DNNVariable input(DNN_WIDTH*DNN_HEIGHT*3); 

#define UPSIDEDOWN
#define WIDTH 320
#define HEIGHT 240

File nnbfile;

const int measureButton = 9;
static bool do_action = false;

void CamCB(CamImage img) {

  if (!img.isAvailable()) return;

#ifdef UPSIDEDOWN
  // カメラ画像の上下左右反転　（カメラが逆さまのため）
  uint16_t *buf = (uint16_t*)img.getImgBuff();
  static uint8_t tmp0[WIDTH*2];
  static uint8_t tmp1[WIDTH*2];
  for (int y = 0; y < HEIGHT/2; ++y) {
    memcpy(&tmp1[0], &buf[y*WIDTH], sizeof(uint16_t)*WIDTH);
    memcpy(&buf[y*WIDTH], &buf[(HEIGHT-y-1)*WIDTH], sizeof(uint16_t)*WIDTH);
    memcpy(&buf[(HEIGHT-y-1)*WIDTH], &tmp1[0], sizeof(uint16_t)*WIDTH);
    memcpy(&tmp0[0], &buf[y*WIDTH], sizeof(uint16_t)*WIDTH);
    uint8_t *buf0 = (uint8_t*)&buf[y*WIDTH];
    uint8_t *buf1 = (uint8_t*)&buf[(HEIGHT-y-1)*WIDTH];
    for (int x = 0; x < WIDTH*2; x += 2) {
      buf0[WIDTH*2-(x-1)-1] = tmp0[x];
      buf0[WIDTH*2-(x-0)-1] = tmp0[x+1];
      buf1[WIDTH*2-(x-1)-1] = tmp1[x];
      buf1[WIDTH*2-(x-0)-1] = tmp1[x+1];
    }
  }    
#endif // UPSIDEDOWN

#ifdef USE_SD_CARD
  nnbfile = SD.open("model3.nnb");
#else
  nnbfile = Flash.open("model3.nnb");
#endif
  if (!nnbfile) {
    Serial.println("nnb not found");
    return;
  }

  Serial.println("DNN3 initialize");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.print("Runtime initialization failure. ");
    Serial.println(ret);
    return;
  }
  nnbfile.close();

  // 画像の切り出しと縮小
  CamImage small; 
  CamErr camErr = img.clipAndResizeImageByHW(small
            ,OFFSET_X ,OFFSET_Y 
            ,OFFSET_X+CLIP_WIDTH-1 ,OFFSET_Y+CLIP_HEIGHT-1 
            ,DNN_WIDTH ,DNN_HEIGHT);
  if (!small.isAvailable()) return;

  // 画像をYUVからRGB565に変換
  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565); 
  uint16_t* sbuf = (uint16_t*)small.getImgBuff();

  // RGBのピクセルをフレームに分割
  float* fbuf_r = input.data();
  float* fbuf_g = fbuf_r + DNN_WIDTH*DNN_HEIGHT;
  float* fbuf_b = fbuf_g + DNN_WIDTH*DNN_HEIGHT;
  for (int i = 0; i < DNN_WIDTH*DNN_HEIGHT; ++i) {
   fbuf_r[i] = (float)((sbuf[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
   fbuf_g[i] = (float)((sbuf[i] >>  5) & 0x3F)/63.0; // 0x3F = 64
   fbuf_b[i] = (float)((sbuf[i])       & 0x1F)/31.0; // 0x1F = 31
  }
  
  // 推論を実行
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0); 
 
  // DNNRTの結果をLCDに出力するために画像化
  static uint16_t result_buf[DNN_WIDTH*DNN_HEIGHT];
  for (int i = 0; i < DNN_WIDTH * DNN_HEIGHT; ++i) {
    uint16_t value = output[i] * 0x3F; // 6bit
    if (value > 0x3F) value = 0x3F;
    result_buf[i] = (value << 5);  // Only Green
  }
  
  // 認識対象の横幅と横方向座標を取得
  bool err;
  int16_t s_sx, s_width;
  err = get_sx_and_width_of_region(output, DNN_WIDTH, DNN_HEIGHT, &s_sx, &s_width);
  
  // 認識対象の縦幅と縦方向座標を取得
  int16_t s_sy, s_height;
  int sx, width, sy, height;
  sx = width = sy = height = 0;
  err = get_sy_and_height_of_region(output, DNN_WIDTH, DNN_HEIGHT, &s_sy, &s_height);
  dnnrt.end(); 
  if (!err) {
    Serial.println("detection error");
    goto disp;
  }
  
  // 何も検出できなかった
  if (s_width == 0 || s_height == 0) {
    Serial.println("no detection");
    goto disp;
  }
  
  // 認証対象のボックスと座標をカメラ画像にあわせて拡大
  sx = s_sx * (CLIP_WIDTH/DNN_WIDTH) + OFFSET_X;
  width = s_width * (CLIP_WIDTH/DNN_WIDTH);
  sy = s_sy * (CLIP_HEIGHT/DNN_HEIGHT) + OFFSET_Y;
  height = s_height * (CLIP_HEIGHT/DNN_HEIGHT);

   
disp:
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);  
  buf = (uint16_t*)img.getImgBuff();  

  static uint16_t num_buf[DNN2_WIDTH*DNN2_HEIGHT];
  if (width >= DNN2_WIDTH && height >= DNN2_HEIGHT) {

    static int g_counter = 0;

     // 認証対象のボックスと座標をカメラ画像にあわせて拡大縮小
     float mag_h = float(DNN2_WIDTH)/float(width);
     float mag_v = float(DNN2_HEIGHT)/float(height);
     for (int y = sy; y < sy + height; ++y) {
      int yy = int(mag_v * float(y-sy));
      for (int x = sx; x < sx + width; ++x) {
        int xx = int(mag_h * float(x-sx));
         num_buf[yy*DNN2_WIDTH + xx] = buf[y*WIDTH + x];
      }
    }    

    // メーター領域を推論
#ifdef USE_SD_CARD
    nnbfile = SD.open("model4.nnb");
#else
    nnbfile = Flash.open("model4.nnb");
#endif
    if (!nnbfile) {
      Serial.println("nnb not found");
      return;
    }

    Serial.println("DNN4 initialize");
    int ret = dnnrt.begin(nnbfile);
    if (ret < 0) {
      Serial.print("Runtime initialization failure. ");
      Serial.println(ret);
      return;
    }
    nnbfile.close();

    //// RGBのピクセルをフレームに分割
    fbuf_r = input.data();
    fbuf_g = fbuf_r + DNN2_WIDTH*DNN2_HEIGHT;
    fbuf_b = fbuf_g + DNN2_WIDTH*DNN2_HEIGHT;
    for (int i = 0; i < DNN2_WIDTH*DNN2_HEIGHT; ++i) {
      fbuf_r[i] = (float)((num_buf[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
      fbuf_g[i] = (float)((num_buf[i] >>  5) & 0x3F)/63.0; // 0x3F = 63
      fbuf_b[i] = (float)((num_buf[i])       & 0x1F)/31.0; // 0x1F = 31
    }

    //// 推論を実行
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    DNNVariable output = dnnrt.outputVariable(0); 

    //// 認識対象の横幅と横方向座標を取得
    bool err;
    int16_t s_sx2, s_width2;
    err = get_sx_and_width_of_region(output, DNN2_WIDTH, DNN2_HEIGHT, &s_sx2, &s_width2);
    
    //// 認識対象の縦幅と縦方向座標を取得
    int16_t s_sy2, s_height2;
    err = get_sy_and_height_of_region(output, DNN2_WIDTH, DNN2_HEIGHT, &s_sy2, &s_height2);
    if (!err) {
      Serial.println("detection error");
      goto disp;
    }

    dnnrt.end();

    // 認証対象のメーターエリアの画像をカメラ画像の座標値に変換
    int sx2, width2, sy2, height2;
    sx2 = width2 = sy2 = height2 = 0;

    int margin = 5;     
    sx2 = sx + s_sx2*width/DNN2_WIDTH;
    width2 = s_width2*width/DNN2_WIDTH + margin;
    sy2 = sy + s_sy2*height/DNN2_HEIGHT;
    height2 = s_height2*height/DNN2_HEIGHT + margin;
    printf("width = %d  height = %d\n", width2, height2);

    draw_box_g(buf, sx2, sy2, width2, height2);

    // メーター領域を格納
    uint16_t *num_buf2 = (uint16_t*)malloc(width2*height*2*sizeof(uint16_t));
    int n = 0;
    for (int y = sy2; y < sy2 + height2; ++y) {
      for (int x = sx2; x < sx2 + width2; ++x) {
        num_buf2[n++] = buf[y*WIDTH + x];
      }
    }

    // メーターの位置を確認するためにX軸の輝度情報を算出
    float *num_th = (float*)malloc(width2*sizeof(float));
    float *ave_th = (float*)malloc(width2*sizeof(float));
    printf("x, y_sum\n");
    for (int x = 0; x < width2; ++x) {
      float r_sum = 0.0;
      float g_sum = 0.0;
      float b_sum = 0.0;
      for (int y = 0; y < height2; ++y) {
        uint16_t value = num_buf2[y*width2 + x];
        r_sum += (float)((value >> 11) & 0x1F)/31.0; // 0x1F = 31
        g_sum += (float)((value >>  5) & 0x3F)/63.0; // 0x3F = 63
        b_sum += (float)((value)       & 0x1F)/31.0; // 0x1F = 31
      }
      float y_sum = 0.299*r_sum + 0.587*g_sum + 0.114*b_sum;
      num_th[x] = y_sum;
    }

    // 輝度情報をスムージング
    const int tap = 5;
    memcpy(&ave_th[0], &num_th[0], sizeof(float)*tap);
    for (int i = tap; i < width2 -tap; ++i) {
      float sum = 0;
      for (int j = -tap; j <= tap; ++j) {
        sum += num_th[i+j];
      }
      ave_th[i] = sum/(2*tap+1);
    }
    memcpy(&ave_th[width2-1-tap], &num_th[width2-1-tap], sizeof(float)*tap);

    // 輝度情報の微分値を算出
    for (int i = 1; i < width2; ++i) {
      num_th[i] = ave_th[i] - ave_th[i-1];
    }

    // 微分値がマイナスからプラスになる位置が境界として位置を測定
    int count = 0;
    int pix_count = 0;
    uint8_t distance[5];
    for (int i = 2; i < width2-2; ++i) {
      bool cond_0 = (num_th[i-2] < 0) && (num_th[i-1] < 0);
      bool cond_1 = (num_th[i-1] < 0) && (num_th[i]  >= 0);
      bool cond_2 = (num_th[i+1] > 0) && (num_th[i+2] > 0);
      if (cond_0 && cond_1 && cond_2) {
        distance[count] = pix_count;
        ++count; pix_count = 0;
      }
      ++pix_count;
    }

    // 境界値が４つ以上あればメーターありと判断（本来は５つだが、４つあれば成功とする）
    printf("count = %d\n", count);
    int ave_distance = 0;
    if (count >= 4) {
      for (int i = 0; i < 5; ++i) {
        printf("%d %d\n", i, distance[i]);
      }
      ave_distance = (distance[2] + distance[3]) / 2;
      printf("ave distance: %d  height: %d\n", ave_distance, height2);

      // １つ目の数字
      int p_x1 = sx2 + distance[0];
      // ２つ目の数字
      int p_x2 = sx2 + distance[0] + ave_distance*1;
      // ３つ目の数字
      int p_x3 = sx2 + distance[0] + ave_distance*2;
      // ４つ目の数字
      int p_x4 = sx2 + distance[0] + ave_distance*3;

      // ボックスを描画
      draw_box(buf, p_x1, sy2, ave_distance, height2);
      draw_box(buf, p_x2, sy2, ave_distance, height2);
      draw_box(buf, p_x3, sy2, ave_distance, height2);
      draw_box(buf, p_x4, sy2, ave_distance, height2);

      // シャッターボタン読み取り
      // do_action = digitalRead(measureButton) ? false : true;
      // Serial.println("do_action is " + String(do_action));

      // メーター領域の横幅が推論用画像の横幅より大きければ処理を行う
      if (ave_distance >= DNN3_WIDTH) {
        static uint16_t num_img[DNN3_WIDTH*DNN3_HEIGHT];
        float mag_h2 = float(DNN3_WIDTH) / ave_distance;
        float mag_v2 = float(DNN3_HEIGHT) / height2;

#ifndef SAVE_NUM_IMAGE
        static const char label[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
        char num_result[4];
        float probability[4];
        float max_value = 0.0;
        // 数字認識モデルをロード
#ifdef USE_SD_CARD
        nnbfile = SD.open("model5.nnb");
#else
        nnbfile = Flash.open("model5.nnb");
#endif
        if (!nnbfile) {
          Serial.println("nnb not found");
          return;
        }

        Serial.println("DNN5 initialize");
        int ret = dnnrt.begin(nnbfile);
        if (ret < 0) {
          Serial.print("Runtime initialization failure. ");
          Serial.println(ret);
          return;
        }
        nnbfile.close();
#endif


        // １つ目の数字
        if (do_action) {
#if defined(SAVE_NUM_IMAGE) && defined(USE_SD_CARD)
          //// データセット用数字画像保存
          save_img(num_img, DNN3_WIDTH, DNN3_HEIGHT);
#else  // 認識モード 
          copy_number_area(buf, num_img, p_x1, sy2, ave_distance, height2, mag_h2, mag_v2);
          //// RGBのピクセルをフレームに分割
          fbuf_r = input.data();
          fbuf_g = fbuf_r + DNN3_WIDTH*DNN3_HEIGHT;
          fbuf_b = fbuf_g + DNN3_WIDTH*DNN3_HEIGHT;
          for (int i = 0; i < DNN3_WIDTH*DNN3_HEIGHT; ++i) {
            fbuf_r[i] = (float)((num_img[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
            fbuf_g[i] = (float)((num_img[i] >>  5) & 0x3F)/63.0; // 0x3F = 63
            fbuf_b[i] = (float)((num_img[i])       & 0x1F)/31.0; // 0x1F = 31
          }

          //// 推論を実行
          dnnrt.inputVariable(input, 0);
          dnnrt.forward();
          DNNVariable output0 = dnnrt.outputVariable(0);
          int index0 = output0.maxIndex();
          num_result[0] = label[index0];
          probability[0] = output0[index0];
          printf("result[0] %d %f\n", index0, probability[index0]);
#endif
        }

  
        // ２つ目の数字
        if (do_action) {
#if defined(SAVE_NUM_IMAGE) && defined(USE_SD_CARD)
          //// データセット用数字画像保存
          save_img(num_img, DNN3_WIDTH, DNN3_HEIGHT);
#else // 認識モード
        copy_number_area(buf, num_img, p_x2, sy2, ave_distance, height2, mag_h2, mag_v2);
        //// RGBのピクセルをフレームに分割
          fbuf_r = input.data();
          fbuf_g = fbuf_r + DNN3_WIDTH*DNN3_HEIGHT;
          fbuf_b = fbuf_g + DNN3_WIDTH*DNN3_HEIGHT;
          for (int i = 0; i < DNN3_WIDTH*DNN3_HEIGHT; ++i) {
            fbuf_r[i] = (float)((num_img[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
            fbuf_g[i] = (float)((num_img[i] >>  5) & 0x3F)/63.0; // 0x3F = 63
            fbuf_b[i] = (float)((num_img[i])       & 0x1F)/31.0; // 0x1F = 31
          }

          //// 推論を実行
          dnnrt.inputVariable(input, 0);
          dnnrt.forward();
          DNNVariable output1 = dnnrt.outputVariable(0); 
          int index1 = output1.maxIndex();
          num_result[1] = label[index1];
          probability[1] = output1[index1];          
          printf("result[1] %d %f\n", index1, probability[index1]);
#endif        
        }


        // ３つ目の数字
        if (do_action) {
#if defined(SAVE_NUM_IMAGE) && defined(USE_SD_CARD)
          //// データセット用数字画像保存
          save_img(num_img, DNN3_WIDTH, DNN3_HEIGHT);
#else // 認識モード
          copy_number_area(buf, num_img, p_x3, sy2, ave_distance, height2, mag_h2, mag_v2);
          //// RGBのピクセルをフレームに分割
          fbuf_r = input.data();
          fbuf_g = fbuf_r + DNN3_WIDTH*DNN3_HEIGHT;
          fbuf_b = fbuf_g + DNN3_WIDTH*DNN3_HEIGHT;
          for (int i = 0; i < DNN3_WIDTH*DNN3_HEIGHT; ++i) {
            fbuf_r[i] = (float)((num_img[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
            fbuf_g[i] = (float)((num_img[i] >>  5) & 0x3F)/63.0; // 0x3F = 63
            fbuf_b[i] = (float)((num_img[i])       & 0x1F)/31.0; // 0x1F = 31
          }

          //// 推論を実行
          dnnrt.inputVariable(input, 0);
          dnnrt.forward();
          DNNVariable output2 = dnnrt.outputVariable(0); 
          int index2 = output2.maxIndex();
          num_result[2] = label[index2];
          probability[2] = output2[index2];
          printf("result[2] %d %f\n", index2, probability[index2]);
#endif        
        }


        // ４つ目の数字
        if (do_action) {
#if defined(SAVE_NUM_IMAGE) && defined(USE_SD_CARD)
          //// データセット用数字画像保存
          save_img(num_img, DNN3_WIDTH, DNN3_HEIGHT);
#else // 認識モード
          copy_number_area(buf, num_img, p_x4, sy2, ave_distance, height2, mag_h2, mag_v2);
          //// RGBのピクセルをフレームに分割
          fbuf_r = input.data();
          fbuf_g = fbuf_r + DNN3_WIDTH*DNN3_HEIGHT;
          fbuf_b = fbuf_g + DNN3_WIDTH*DNN3_HEIGHT;
          for (int i = 0; i < DNN3_WIDTH*DNN3_HEIGHT; ++i) {
            fbuf_r[i] = (float)((num_img[i] >> 11) & 0x1F)/31.0; // 0x1F = 31
            fbuf_g[i] = (float)((num_img[i] >>  5) & 0x3F)/63.0; // 0x3F = 63
            fbuf_b[i] = (float)((num_img[i])       & 0x1F)/31.0; // 0x1F = 31
          }

          //// 推論を実行
          dnnrt.inputVariable(input, 0);
          dnnrt.forward();
          DNNVariable output3 = dnnrt.outputVariable(0); 
          int index3 = output3.maxIndex();
          num_result[3] = label[index3];
          probability[3] = output3[index3];
          printf("result[3] %d %f\n", index3, probability[index3]);
#endif
        }



        dnnrt.end();

#ifndef SAVE_NUM_IMAGE
        if (do_action) {
          // 認識結果をLCDに表示
          printf("%c%c%c%c\n", num_result[0], num_result[1], num_result[2], num_result[3]);
          float ave_possiblity = (probability[0] + probability[1] + probability[2] + probability[3]) / 4;
          String str = String(num_result[0]) + String(num_result[1]) + String(num_result[2]) + String(num_result[3]) + " " + String(ave_possiblity);
          int len = str.length();
          tft.fillRect(DNN_WIDTH,0, WIDTH-DNN_WIDTH*2, DNN_HEIGHT, ILI9341_BLUE);
          tft.setTextSize(2);
          tft.setCursor(100, 20);
          tft.setTextColor(ILI9341_YELLOW);
          tft.println(str);        
          delay(3000);
          tft.fillRect(DNN_WIDTH,0, WIDTH-DNN_WIDTH*2, DNN_HEIGHT, ILI9341_BLUE);
        }
#endif
        do_action = false;
      }
    }


    // 確保したメモリを開放
    memset(num_buf2, 0, width2*height2*sizeof(uint16_t)); free(num_buf2); num_buf2 = NULL;
    memset(num_th, 0, width2*sizeof(float)); free(num_th); num_th = NULL;
    memset(ave_th, 0, width2*sizeof(float)); free(ave_th); ave_th = NULL;    
  }
  
  // サイドバンドをフレームバッファに描画
  draw_sideband(buf, OFFSET_X, ILI9341_BLACK);
    
  // DNNRTへの入力画像をLCDの左上に表示
  tft.drawRGBBitmap(0, 0, (uint16_t*)sbuf, DNN_WIDTH, DNN_HEIGHT);  
  // DNNRTの出力画像をLCDの右上に表示
  tft.drawRGBBitmap(320-DNN_WIDTH, 0, result_buf, DNN_WIDTH, DNN_HEIGHT);  
  // ボックス描画されたカメラ画像を表示
  tft.drawRGBBitmap(0, DNN_HEIGHT, buf, 320, 240-DNN_HEIGHT);
}

void copy_number_area(uint16_t *buf, uint16_t *num_img, const int sx, const int sy, const int width, const int height, const float mag_h, const float mag_v) {
    for (int y = sy; y < sy + height; ++y) {
      int yy = int(float(y - sy)*mag_v);
      for (int x = sx; x < sx + width; ++x) {
        int xx = int(float(x - sx)*mag_h);
        num_img[yy*DNN3_WIDTH + xx] = buf[y*WIDTH + x];
      }
    }
}


#if defined(SAVE_NUM_IMAGE) && defined(USE_SD_CARD)
void save_img(uint16_t *buf, int width, int height) {
  static int g_count = 0;
  char filename[16] = {0};

  sprintf(filename, "bmp%03d.bmp", g_count++);
  if (SD.exists(filename)) SD.remove(filename);
  File myFile = SD.open(filename, FILE_WRITE);

  bmp.begin(BmpImage::BMP_IMAGE_RGB565, width, height, (uint8_t*)buf);
  bmp.alignImageLine(false);  // need 32bts alignment before saving the image
  myFile.write(bmp.getBmpBuff(), bmp.getBmpSize());
  printf("save number image: %s\n", filename);
  myFile.close();
  bmp.end();   
}
#endif

void bShutter() {
  do_action = true;
}

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  tft.fillRect(0, 0, 320, 240, ILI9341_BLUE);
  
  pinMode(measureButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(measureButton), bShutter, FALLING);
  theCamera.begin();
  theCamera.startStreaming(true, CamCB);
}

void loop() {
  // put your main code here, to run repeatedly:

}
