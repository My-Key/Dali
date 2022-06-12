#include "DaliWatchy.h"

const float VOLTAGE_MIN = 3.5;
const float VOLTAGE_MAX = 4.2;
const float VOLTAGE_WARNING = 3.6;
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;

const int DENSITY = 1;
const int VECTOR_SIZE = 60 * DENSITY;
const double STEP_ANGLE = 360 / VECTOR_SIZE;

const int STEP_MINUTE = DENSITY;
const int STEP_HOUR = VECTOR_SIZE/12;

const Vector CENTER = {99.5, 99.5};
const int RADIUS = 99;
const int RIM_SIZE = 12;
const int FACE_RADIUS = 100 - RIM_SIZE;

const double BATTERY_MIN = 0.5;
const double BATTERY_RANGE = 1.0 - BATTERY_MIN;
const double BATTERY_WARNING = BATTERY_MIN + ((VOLTAGE_WARNING - VOLTAGE_MIN) / VOLTAGE_RANGE) * BATTERY_RANGE;

const Vector HAND[] =
{{0, -1.0},
 {0, -0.8}, 
 {0.15, -0.8},
{0, 0},
 {0.05, 0.15},
{-0.05, 0.15},
 {-0.15, -0.8}};

const Vector HAND_NORMAL[] =
{{0.5, -0.85}, {0.2, -0.2}, {0.85, -0.5},
{0.3, -0.1}, {0.96, -0.1}, {0.3, 0.1}, {0.96, 0.1},
{0, 0.3}, {0.1, 0.96}, {-0.1, 0.96},
{-0.3, -0.1}, {-0.96, -0.1}, {-0.3, 0.1}, {-0.96, 0.1},
{-0.5, -0.85}, {-0.2, -0.2}, {-0.85, -0.5}};

const int HAND_POS_INDEX[] = 
{0,1,2,
1,2,3,
2,3,4,
3,4,5,
3,5,6,
3,6,1,
6,1,0};

const int HAND_POS_LEN = 7;

const int HAND_NORMAL_INDEX[] = 
{0,1,2,
3,4,5,
4,5,6,
7,8,9,
10,11,13,
10,13,12,
14,15,16};

const int HAND_OUTLINE_INDEX[] = {0,2,4,5,6};

const int HAND_OUTLINE_LEN = 5;

Vector EDGE_VECTORS[VECTOR_SIZE];
Vector EDGE_NORMAL[VECTOR_SIZE];

DaliWatchy::DaliWatchy(const watchySettings& s) : Watchy(s)
{
  Vector up = {0.0, -1.0};
  
  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    EDGE_NORMAL[i] = EDGE_VECTORS[i] = Vector::rotateVector(up, i * STEP_ANGLE);
    EDGE_NORMAL[i].normalize();
    EDGE_VECTORS[i].normalize();
  }
}

static void RecalculateNormal(double scale[], Vector normals[])
{
  int startIndex = 0;
  int prevIndex = -1 + VECTOR_SIZE;

  Vector prevVector = EDGE_VECTORS[prevIndex] * scale[prevIndex];
  Vector startVector = EDGE_VECTORS[startIndex] * scale[startIndex];
  
  Vector prevNormal = prevVector - startVector;
  prevNormal.normalize();
  prevNormal = Vector::rotateVectorByRightAngle(prevNormal, 1);

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    int nextIndex = (i + 1) % VECTOR_SIZE;

    Vector v1 = EDGE_VECTORS[i] * scale[i];
    Vector v2 = EDGE_VECTORS[nextIndex] * scale[nextIndex];

    Vector nextNormal = v1 - v2;
    nextNormal.normalize();
    nextNormal = Vector::rotateVectorByRightAngle(nextNormal, 1);

    Vector normal = prevNormal + nextNormal;
    normal.normalize();

    prevNormal = nextNormal;

    normals[i] = normal;
  }
}

void DaliWatchy::drawWatchFace()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  double scale[VECTOR_SIZE];
  Vector normals[VECTOR_SIZE];
  Vector normals2[VECTOR_SIZE];

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    normals[i] = EDGE_NORMAL[i];
    normals2[i] = EDGE_NORMAL[i];
  }

  int hour = currentTime.Hour;
  int minute = currentTime.Minute;
  
  double minuteNormalized = minute / 60.0;
  double hourNormalized = (hour + minuteNormalized) / 24.0;

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    double value = (double)i / VECTOR_SIZE;

    scale[i] = 0.8 
    + sin((value + hourNormalized * 2) * PI * 4) * 0.05
    + sin((value + -hourNormalized * 12) * PI * 6) * 0.075
    + sin((value + minuteNormalized) * PI * 10) * 0.075;
  }

  RecalculateNormal(scale, normals2);

  double batteryFill = getBatteryFill();
  double batteryFillScale = BATTERY_MIN + BATTERY_RANGE * batteryFill;
  double rimSize = RIM_SIZE * batteryFillScale;

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    int nextIndex = (i + 1) % VECTOR_SIZE;

    double scale1 = FACE_RADIUS * scale[i];
    Vector v1 = EDGE_VECTORS[i] * scale1 + CENTER;
    Vector uv1 = normals[i] * RADIUS + CENTER;

    double scale2 = FACE_RADIUS * scale[nextIndex];
    Vector v2 = EDGE_VECTORS[nextIndex] * scale2 + CENTER;
    Vector uv2 = normals[nextIndex] * RADIUS + CENTER;

    fillTriangle(CENTER, CENTER, v1, uv1, v2, uv2, epd_bitmap_Dali_face, 200, 200);

    Vector v4 = EDGE_VECTORS[i] * (scale1 + rimSize) + CENTER;
    Vector uv3 = normals2[i] * -RADIUS + CENTER;
    Vector uv4 = normals2[i] * RADIUS + CENTER;

    Vector v6 = EDGE_VECTORS[nextIndex] * (scale2 + rimSize) + CENTER;
    Vector uv5 = normals2[nextIndex] * -RADIUS + CENTER;
    Vector uv6 = normals2[nextIndex] * RADIUS + CENTER;

    fillTriangle2(v1, uv3, v4, uv4, v2, uv5, MatCapSource, 200, 200);
    fillTriangle2(v4, uv4, v2, uv5, v6, uv6, MatCapSource, 200, 200);

    display.drawLine(v1.x, v1.y, v2.x, v2.y, GxEPD_BLACK);
    display.drawLine(v4.x, v4.y, v6.x, v6.y, GxEPD_BLACK);
  }

  double hourAngle = ((double)(hour % 12) + minuteNormalized) * 30;

  DrawHand(hourAngle, 45);
  DrawHand(minute * 6, 65);
}

void DaliWatchy::DrawHand(double angle, double size)
{
  double radians = angle * DEG_TO_RAD;
  double sinAngle = sin(radians);
  double cosAngle = cos(radians);

  for (int i = 0; i < HAND_POS_LEN; i++)
  {
    Vector v1 = Vector::rotateVector(HAND[HAND_POS_INDEX[i * 3]], sinAngle, cosAngle) * size + CENTER;
    Vector v2 = Vector::rotateVector(HAND[HAND_POS_INDEX[i * 3 + 1]], sinAngle, cosAngle) * size + CENTER;
    Vector v3 = Vector::rotateVector(HAND[HAND_POS_INDEX[i * 3 + 2]], sinAngle, cosAngle) * size + CENTER;

    Vector uv1 = Vector::rotateVector(HAND_NORMAL[HAND_NORMAL_INDEX[i * 3]], sinAngle, cosAngle) * 99 + CENTER;
    Vector uv2 = Vector::rotateVector(HAND_NORMAL[HAND_NORMAL_INDEX[i * 3 + 1]], sinAngle, cosAngle) * 99 + CENTER;
    Vector uv3 = Vector::rotateVector(HAND_NORMAL[HAND_NORMAL_INDEX[i * 3 + 2]], sinAngle, cosAngle) * 99 + CENTER;

    fillTriangle2(v1, uv1, v2, uv2, v3, uv3, MatCapSource, 200, 200);
  }

  for (int i = 0; i < HAND_OUTLINE_LEN; i++)
  {
    Vector v1 = Vector::rotateVector(HAND[HAND_OUTLINE_INDEX[i]], sinAngle, cosAngle) * size + CENTER;
    Vector v2 = Vector::rotateVector(HAND[HAND_OUTLINE_INDEX[(i + 1) % HAND_OUTLINE_LEN]], sinAngle, cosAngle) * size + CENTER;

    display.drawLine(v1.x, v1.y, v2.x, v2.y, GxEPD_BLACK);
  }
}

double DaliWatchy::getBatteryFill()
{
  float VBAT = getBatteryVoltage();

  // 12 battery states
  double batState = ((VBAT - VOLTAGE_MIN) / VOLTAGE_RANGE);
  if (batState > 1.0)
    batState = 1.0;
  if (batState < 0)
    batState = 0;

  return batState;
}

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef _swap_vector
#define _swap_vector(a, b)                                                    \
  {                                                                            \
    Vector t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef _swap_vector_int
#define _swap_vector_int(a, b)                                                    \
  {                                                                            \
    VectorInt t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

static double clamp(double val, double min, double max)
{
  if (val > max)
    val = max;
  
  if (val < min)
    val = min;

  return val;
}

static void barycentric(VectorInt p, VectorInt v0, VectorInt v1, VectorInt a, double invDen, double &u, double &v, double &w)
{
    VectorInt v2 = p - a;
    // ToDo: Premultiply v0 and v1 by invDen?
    v = (v2.x * v1.y - v1.x * v2.y) * invDen;
    w = (v0.x * v2.y - v2.x * v0.y) * invDen;
    u = 1.0 - v - w;
}


static bool getColor(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  int16_t byteWidth = (w + 7) / 8;
  return (pgm_read_byte(bitmap + y * byteWidth + x / 8) & (128 >> (x & 7)));
}

void DaliWatchy::drawLine(int x, int y, int w, VectorInt v0, Vector uv0, VectorInt a, Vector uv1, VectorInt b, Vector uv2, double invDen, const uint8_t *bitmap, int16_t bw, int16_t bh)
{
  for (int i = 0; i < w; i++)
  {
    double ua, va, wa;
    VectorInt pointA = {x + i, y};
    barycentric(pointA, a, b, v0, invDen, ua, va, wa);

    Vector uv = uv0 * ua + uv1 * va + uv2 * wa;

    bool white = getColor(uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }
}

void DaliWatchy::fillTriangle(VectorInt v0, Vector uv0, VectorInt v1, Vector uv1, VectorInt v2, Vector uv2, const uint8_t bitmap[], int w, int h)
{
  int16_t a, b, y, last;
  Vector uvA, uvB;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }
  if (v1.y > v2.y) {
    _swap_vector_int(v2, v1);
    _swap_vector(uv2, uv1);
  }
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }

  display.startWrite();
  if (v0.y == v2.y) { // Handle awkward all-on-same-line case as its own thing
    a = b = v0.x;
    uvA = uv0;
    uvB = uv0;

    if (v1.x < a)
    {
      a = v1.x;
      uvA = uv1;
    }
    else if (v1.x > b)
    { 
      b = v1.x;
      uvB = uv1;
    }
    if (v2.x < a)
    {
      a = v2.x;
      uvA = uv2;
    }
    else if (v2.x > b)
    {
      b = v2.x;
      uvB = uv2;
    }

    writeFastHLineUV(a, v0.y, b - a + 1, uvA, uvB, bitmap, w, h);
    display.endWrite();
    return;
  }

  int16_t dx01 = v1.x - v0.x, dy01 = v1.y - v0.y, 
          dx02 = v2.x - v0.x, dy02 = v2.y - v0.y,
          dx12 = v2.x - v1.x, dy12 = v2.y - v1.y;
  int32_t sa = 0, sb = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (v1.y == v2.y)
    last = v1.y; // Include y1 scanline
  else
    last = v1.y - 1; // Skip it

    
  VectorInt aa = v1 - v0, bb = v2 - v0;
  double invDen = 1 / VectorInt::crossProduct(aa, bb);

  for (y = v0.y; y <= last; y++) {
    a = v0.x + sa / dy01;
    b = v0.x + sb / dy02;

    sa += dx01;
    sb += dx02;

    
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = (int32_t)dx12 * (y - v1.y);
  sb = (int32_t)dx02 * (y - v0.y);
  for (; y <= v2.y; y++) {
    a = v1.x + sa / dy12;
    b = v0.x + sb / dy02;

    sa += dx12;
    sb += dx02;

    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }
  display.endWrite();
}

void DaliWatchy::writeFastHLineUV(int16_t x, int16_t y, int16_t w, Vector uvA, Vector uvB, const uint8_t bitmap[], int bw, int bh)
{
  display.startWrite();

  for (int i = 0; i < w; i++)
  {
    double lerpVal = i / (w + 1.0);
    Vector uv = (uvA * lerpVal) + (uvB * (1.0 - lerpVal));
    bool white = getColor(uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }

  display.endWrite();
}

static bool getColor2(int16_t x, int16_t y, int16_t xUv, int16_t yUv, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  return bitmap[yUv * w + xUv] > BlueNoise200[y * 200 + x];
}

void DaliWatchy::drawLine2(int x, int y, int w, VectorInt v0, Vector uv0, VectorInt a, Vector uv1, VectorInt b, Vector uv2, double invDen, const uint8_t *bitmap, int16_t bw, int16_t bh)
{
  for (int i = 0; i < w; i++)
  {
    double ua, va, wa;
    VectorInt pointA = {x + i, y};
    barycentric(pointA, a, b, v0, invDen, ua, va, wa);

    Vector uv = uv0 * ua + uv1 * va + uv2 * wa;

    bool white = getColor2(x + i, y, uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }
}

void DaliWatchy::fillTriangle2(VectorInt v0, Vector uv0, VectorInt v1, Vector uv1, VectorInt v2, Vector uv2, const uint8_t bitmap[], int w, int h)
{
  int16_t a, b, y, last;
  Vector uvA, uvB;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }
  if (v1.y > v2.y) {
    _swap_vector_int(v2, v1);
    _swap_vector(uv2, uv1);
  }
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }

  display.startWrite();
  if (v0.y == v2.y) { // Handle awkward all-on-same-line case as its own thing
    a = b = v0.x;
    uvA = uv0;
    uvB = uv0;

    if (v1.x < a)
    {
      a = v1.x;
      uvA = uv1;
    }
    else if (v1.x > b)
    { 
      b = v1.x;
      uvB = uv1;
    }
    if (v2.x < a)
    {
      a = v2.x;
      uvA = uv2;
    }
    else if (v2.x > b)
    {
      b = v2.x;
      uvB = uv2;
    }

    writeFastHLineUV2(a, v0.y, b - a + 1, uvA, uvB, bitmap, w, h);
    display.endWrite();
    return;
  }

  int16_t dx01 = v1.x - v0.x, dy01 = v1.y - v0.y, 
          dx02 = v2.x - v0.x, dy02 = v2.y - v0.y,
          dx12 = v2.x - v1.x, dy12 = v2.y - v1.y;
  int32_t sa = 0, sb = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (v1.y == v2.y)
    last = v1.y; // Include y1 scanline
  else
    last = v1.y - 1; // Skip it

    
  VectorInt aa = v1 - v0, bb = v2 - v0;
  double invDen = 1 / VectorInt::crossProduct(aa, bb);

  for (y = v0.y; y <= last; y++) {
    a = v0.x + sa / dy01;
    b = v0.x + sb / dy02;

    sa += dx01;
    sb += dx02;

    
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine2(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = (int32_t)dx12 * (y - v1.y);
  sb = (int32_t)dx02 * (y - v0.y);
  for (; y <= v2.y; y++) {
    a = v1.x + sa / dy12;
    b = v0.x + sb / dy02;

    sa += dx12;
    sb += dx02;

    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine2(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }
  display.endWrite();
}

void DaliWatchy::writeFastHLineUV2(int16_t x, int16_t y, int16_t w, Vector uvA, Vector uvB, const uint8_t bitmap[], int bw, int bh)
{
  display.startWrite();

  for (int i = 0; i < w; i++)
  {
    double lerpVal = i / (w + 1.0);
    Vector uv = (uvA * lerpVal) + (uvB * (1.0 - lerpVal));
    bool white = getColor2(x + i, y, uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }

  display.endWrite();
}
