/*
  Copyright 2025 Seiko Epson Corporation

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <map>
#include <string>
#include <vector>

using namespace std;

struct PickPlacePoint
{
    vector<double> home;
    vector<double> p1_point;
    vector<double> p2_point;
};

//Target pose of the robot in the demo program
class EpsonRBPickPlacePoint{
    private:
    PickPlacePoint GX4_C251S_Point={{0.80285,1.4835,0,0},{-0.034907,1.0647,-0.085,0.78540},{2.0944,1.0647,-0.085,0.78540}};
    PickPlacePoint GX4_C301S_Point={{0.76795,2.0944,-0.021,0},{0.27925,1.0647,-0.084,3.1416},{1.9548,1.0647,-0.084,3.1416}};
    PickPlacePoint GX4_C351S_Point={{0.872666,1.5708,-0.024,0},{0.61087,0.78540,-0.097,1.5708},{2.0595,0.78540,-0.097,1.5708}};
    PickPlacePoint GX4_C351S_R_Point={{1.0297,1.5708,-0.016,0},{0.52360,0.75049,-0.09,-3.1416},{2.0595,0.75049,-0.09,-3.1416}};
    PickPlacePoint GX4_C351S_L_Point={{0.95993,1.8850,-0.038,0},{0.26180,1.2217,-0.09,-1.5708},{0.26180,1.2217,-0.09,-1.5708}};
    PickPlacePoint C8_C901S_Point={{0,-0.24435,-0.20944,0,-0.83776,0},{-0.62832,-1.1170,0.38397,0,-0.83776,0},{0.62832,-1.1170,0.38397,0,-0.83776,0}};
    PickPlacePoint C8_C1401S_Point={{0,-0.17453,-0.55851,0,-0.57596,0},{-0.45379,-0.82030,-0.22689,0,-0.57596,0},{0.45379,-0.82030,-0.22689,0,-0.57596,0}};
    PickPlacePoint C12_C1401S_Point={{0,-0.17453,-0.75049,0,-0.33161,0},{-0.66322,-0.83776,-0.34907,0,-0.33161,0},{0.66322,-0.83776,-0.34907,0,-0.33161,0}};

    map<string,PickPlacePoint> AllRBPickPalcePoint=
    {
        {"GX4-C251S", GX4_C251S_Point},
        {"GX4-C301S", GX4_C301S_Point},
        {"GX4-C351S", GX4_C351S_Point},
        {"GX4-C351S-R", GX4_C351S_R_Point},
        {"GX4-C351S-L", GX4_C351S_L_Point},
        {"C8-C901S", C8_C901S_Point},
        {"C8-C1401S", C8_C1401S_Point},
        {"C12-C1401S", C12_C1401S_Point}
        
    };


    public:
    EpsonRBPickPlacePoint();
    ~EpsonRBPickPlacePoint();
    PickPlacePoint LoadPickPlacePoint(const string& rb_model) {return AllRBPickPalcePoint[rb_model];};
    bool CanLoadPickPlacePoint(const string& rb_model);

};