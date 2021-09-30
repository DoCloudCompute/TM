#include <iostream>
#include <vector>
#include <tuple>
#include "vec_tools.cpp"

using namespace std;

tuple< vector<float> > make_screen(float screen_distance, vector<float> resolution) {
  vector<float> corner;
  vector<float> edge1;
  vector<float> edge2;

  corner.push_back(screen_distance);
  corner.push_back(resolution[0]/2);
  corner.push_back(resolution[1]/2);

  edge1.push_back(0);
  edge1.push_back(-1);
  edge1.push_back(0);

  edge2.push_back(0);
  edge2.push_back(0);
  edge2.push_back(-1);

  return make_tuple(corner, edge1, edge2);
}

int main(int argc, char *argv[]) {
  vector<float> resolution;
  float screen_distance;

  screen_distance = 2330;

  resolution.push_back(720);
  resolution.push_back(720);



  //tuple<vector<float>> result;
  make_screen(screen_distance, resolution);

  //for (int i = 0; i <= 2; i++){
    cout << "result[0][0]" << endl;
  //}

  return 0;
}