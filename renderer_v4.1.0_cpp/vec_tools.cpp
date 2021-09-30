#include <vector>
#include <math.h>

using namespace std;

vector<float> cross(vector<float> a,  vector<float> b) {
  vector<float> result;
  result.push_back(a[1]*b[2] - a[2]*b[1]);
  result.push_back(a[2]*b[0] - a[0]*b[2]);
  result.push_back(a[0]*b[1] - a[1]*b[0]);
  return result;
}

vector<float> negative(vector<float> a) {
  vector<float> result;
  result.push_back(a[0] * -1);
  result.push_back(a[1] * -1);
  result.push_back(a[2] * -1);
  return result;
}

vector<float> add3(vector<float> a,  vector<float> b) {
  vector<float> result;
  result.push_back(a[0] + b[0]);
  result.push_back(a[1] + b[1]);
  result.push_back(a[2] + b[2]);
  return result;
}

vector<float> sub3(vector<float> a,  vector<float> b) {
  vector<float> result;
  result.push_back(a[0] - b[0]);
  result.push_back(a[1] - b[1]);
  result.push_back(a[2] - b[2]);
  return result;
}

vector<float> div3(vector<float> a,  vector<float> b) {
  vector<float> result;
  result.push_back(a[0] / b[0]);
  result.push_back(a[1] / b[1]);
  result.push_back(a[2] / b[2]);
  return result;
}

vector<float> div3_scalar(vector<float> a, float b) {
  vector<float> result;
  result.push_back(a[0] / b);
  result.push_back(a[1] / b);
  result.push_back(a[2] / b);
  return result;
}

float det3(vector<float> a, vector<float> b, vector<float> c) {
  float result;
  result = a[0]*b[1]*c[2] + b[0]*c[1]*a[2] + c[0]*a[1]*b[2] - c[0]*b[1]*a[2] - b[0]*a[1]*c[2] - a[0]*c[1]*b[2];
  return result;
}

float det2(vector<float> a, vector<float> b){
  float result;
  result = a[0]*b[1] - b[0]*a[1];
  return result;
}

float dot3(vector<float> a, vector<float> b){
  float result;
  result = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  return result;
}

float dot2(vector<float> a, vector<float> b){
  float result;
  result = a[0]*b[0] + a[1]*b[1];
  return result;
}

vector<float> mult3(vector<float> a, vector<float> b) {
  vector<float> result;
  result.push_back(a[0] * b[0]);
  result.push_back(a[1] * b[1]);
  result.push_back(a[2] * b[2]);
  return result;
}

vector<float> mult3_scalar(vector<float> a, float b) {
  vector<float> result;
  result.push_back(a[0] * b);
  result.push_back(a[1] * b);
  result.push_back(a[2] * b);
  return result;
}

vector<float> vec3(vector<float> a, vector<float> b) {
  vector<float> result;
  result = sub3(a, b);
  return result;
}

vector<float> vec2(vector<float> a, vector<float> b) {
  vector<float> result;
  result.push_back(a[0] - b[0]);
  result.push_back(a[1] - b[1]);
  return result;
}

float norm3(vector<float> a){
  float result;
  result = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  return result;
}

float norm3_sq(vector<float> a){
  float result;
  result = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
  return result;
}

float points_separation_sq(vector<float> a, vector<float> b) {
  float result;
  vector<float> vec_BA;
  vec_BA = sub3(a, b);
  result = norm3_sq(vec_BA);
  return result;
}

float points_separation(vector<float> a, vector<float> b) {
  float result;
  vector<float> vec_BA;
  vec_BA = sub3(a, b);
  result = norm3(vec_BA);
  return result;
}

float norm2(vector<float> a){
  float result;
  result = sqrt(a[0]*a[0] + a[1]*a[1]);
  return result;
}

vector<float> unit3(vector<float> a) {
  vector<float> result;
  float vec_len;

  vec_len = norm3(a);
  result = div3_scalar(a, vec_len);

  return result;
}

vector<float> centroid3(vector<float> a,  vector<float> b, vector<float> c) {
  vector<float> result;
  result.push_back((a[0] + b[0] + c[0]) / 3);
  result.push_back((a[1] + b[1] + c[1]) / 3);
  result.push_back((a[0] + b[0] + c[0]) / 3);
  return result;
}

vector<float> mixRGB(vector<float> a, vector<float> b, float f = 0.5) {
  vector<float> result;
  float opp_f;

  opp_f = 1-f;

  result.push_back(a[0]*f + b[0]*opp_f);
  result.push_back(a[1]*f + b[1]*opp_f);
  result.push_back(a[2]*f + b[2]*opp_f);
  return result;
}