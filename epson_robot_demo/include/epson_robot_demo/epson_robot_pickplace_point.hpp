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
    //GX4
    PickPlacePoint GX4_C251X_Point={{0.80285,1.4835,0,0},{-0.034907,1.0647,-0.07,0.78540},{2.0944,1.0647,-0.07,0.78540}};
    PickPlacePoint GX4_C301X_Point={{0.76795,2.0944,-0.021,0},{0.27925,1.0647,-0.07,3.1416},{1.9548,1.0647,-0.07,3.1416}};
    PickPlacePoint GX4_C351X_Point={{0.872666,1.5708,-0.024,0},{0.61087,0.78540,-0.065,1.5708},{1.9548,0.78540,-0.065,1.5708}};
    PickPlacePoint GX4_C351X_R_Point={{1.0297,1.5708,-0.016,0},{0.52360,0.75049,-0.07,-3.1416},{2.0595,0.75049,-0.07,-3.1416}};
    PickPlacePoint GX4_C351X_L_Point={{0.95993,1.8850,-0.038,0},{0.26180,1.2217,-0.07,-1.5708},{2.0071,1.2217,-0.07,-1.5708}};
    //GX8
    PickPlacePoint GX8_C452X_Point={{0.73304,1.4835,0,0},{-0.034907,1.0647,-0.12,0.78540},{2.0944,1.0647,-0.12,0.78540}};
    PickPlacePoint GX8_C453X_Point={{0.73304,1.4835,0,0},{-0.034907,1.0647,-0.10,0.78540},{2.0944,1.0647,-0.10,0.78540}};
    PickPlacePoint GX8_C552X_Point={{0.89012,1.5533,0,0},{-0.034907,1.0647,-0.10,0.78540},{2.0944,1.0647,-0.10,0.78540}};
    PickPlacePoint GX8_C553X_Point={{0.80285,1.8326,0,0},{-0.034907,1.0647,-0.20,0.78540},{2.0944,1.0647,-0.20,0.78540}};
    PickPlacePoint GX8_C652X_Point={{1.0472,1.4835,0,0},{-0.034907,1.0647,-0.10,0.78540},{2.0944,1.0647,-0.10,0.78540}};
    PickPlacePoint GX8_C653X_Point={{1.0472,1.4835,0,0},{-0.034907,1.0647,-0.20,0.78540},{2.0944,1.0647,-0.20,0.78540}};
    //RS4-RS6
    PickPlacePoint RS4_C351X_Point={{0.99484,-2.1293,0,0},{-0.017453,-1.57080,-0.077,0.78540},{2.4784,-3.3161,-0.077,0.78540}};
    PickPlacePoint RS6_C552X_Point={{1.06465,-2.1293,0,0},{-0.017453,-1.57080,-0.1,0.78540},{2.6704,-2.1118,-0.1,0.78540}};
    //C8
    PickPlacePoint C8_C901X_Point={{0,-0.24435,-0.20944,0,-0.83776,0},{-0.62832,-1.1170,0.38397,0,-0.83776,0},{0.62832,-1.1170,0.38397,0,-0.83776,0}};
    PickPlacePoint C8_C1401X_Point={{0,-0.17453,-0.55851,0,-0.57596,0},{-0.45379,-0.82030,-0.22689,0,-0.57596,0},{0.45379,-0.82030,-0.22689,0,-0.57596,0}};
    //C12
    PickPlacePoint C12_C1401X_Point={{0,-0.17453,-0.75049,0,-0.33161,0},{-0.66322,-0.83776,-0.34907,0,-0.33161,0},{0.66322,-0.83776,-0.34907,0,-0.33161,0}};
    //CX4
    PickPlacePoint CX4_A601X_Point={{0,-0.38397,-0.13963,0,-0.89012,0},{-0.62832,-0.85521,0.26180,0,-0.89012,0},{0.62832,-0.85521,0.26180,0,-0.89012,0}};
   //CX7
    PickPlacePoint CX7_A701X_Point={{0,-0.41888,0.29671,0,-1.4137,0},{-0.61087,-0.94248,0.19199,0,-0.66322,0},{0.61087,-0.94248,0.19199,0,-0.66322,0}};
    PickPlacePoint CX7_A901X_Point={{0,-0.33161,0.20944,0,-1.5359,0},{-0.64577,-0.59341,-0.19199,0,-0.64577,0},{0.64577,-0.59341,-0.19199,0,-0.64577,0}};

    map<string,PickPlacePoint> AllRBPickPalcePoint=
    {
        //GX4
        {"GX4-C251S", GX4_C251X_Point},
        {"GX4-C251C", GX4_C251X_Point},
        {"GX4-C301S", GX4_C301X_Point},
        {"GX4-C301SM", GX4_C301X_Point},
        {"GX4-C301C", GX4_C301X_Point},
        {"GX4-C301CM", GX4_C301X_Point},
        {"GX4-C351S", GX4_C351X_Point},
        {"GX4-C351SM", GX4_C351X_Point},
        {"GX4-C351C", GX4_C351X_Point},
        {"GX4-C351CM", GX4_C351X_Point},
        {"GX4-C351S-R", GX4_C351X_R_Point},
        {"GX4-C351C-R", GX4_C351X_R_Point},
        {"GX4-C351S-L", GX4_C351X_L_Point},
        {"GX4-C351C-L", GX4_C351X_L_Point},

        //GX8
        {"GX8-C452S", GX8_C452X_Point},
        {"GX8-C452C", GX8_C452X_Point},
        {"GX8-C452SR",GX8_C452X_Point},
        {"GX8-C452CR", GX8_C452X_Point},
        {"GX8-C453S", GX8_C453X_Point},
        {"GX8-C453C", GX8_C453X_Point},
        {"GX8-C453SR",GX8_C453X_Point},
        {"GX8-C453CR", GX8_C453X_Point},
        {"GX8-C552S", GX8_C552X_Point},
        {"GX8-C552C", GX8_C552X_Point},
        {"GX8-C552SR",GX8_C552X_Point},
        {"GX8-C552CR", GX8_C552X_Point},
        {"GX8-C553S", GX8_C553X_Point},
        {"GX8-C553C", GX8_C553X_Point},
        {"GX8-C553SR",GX8_C553X_Point},
        {"GX8-C553CR", GX8_C553X_Point},
        {"GX8-C652S", GX8_C652X_Point},
        {"GX8-C652C", GX8_C652X_Point},
        {"GX8-C652SR",GX8_C652X_Point},
        {"GX8-C652CR", GX8_C652X_Point},
        {"GX8-C653S", GX8_C653X_Point},
        {"GX8-C653C", GX8_C653X_Point},
        {"GX8-C653SR",GX8_C653X_Point},
        {"GX8-C653CR", GX8_C653X_Point},

        //RS4
        {"RS4-C351S", RS4_C351X_Point},
        {"RS4-C351C", RS4_C351X_Point},

        //RS6
        {"RS6-C552S", RS6_C552X_Point},
        {"RS6-C552C", RS6_C552X_Point},

        //C8
        {"C8-C901S", C8_C901X_Point},
        {"C8-C901C", C8_C901X_Point},
        {"C8-C901SR", C8_C901X_Point},
        {"C8-C901CR", C8_C901X_Point},
        {"C8-C1401S", C8_C1401X_Point},
        {"C8-C1401C", C8_C1401X_Point},
        {"C8-C1401SR", C8_C1401X_Point},
        {"C8-C1401CR", C8_C1401X_Point},

        //C12
        {"C12-C1401S", C12_C1401X_Point},
        {"C12-C1401C", C12_C1401X_Point},
        
        //CX4
        {"CX4-A601S", CX4_A601X_Point},
        {"CX4-A601SR", CX4_A601X_Point},
        {"CX4-A601C", CX4_A601X_Point},
        {"CX4-A601CR", CX4_A601X_Point},
        
        //CX7
        {"CX7-A701S", CX7_A701X_Point},
        {"CX7-A701SR", CX7_A701X_Point},
        {"CX7-A701C", CX7_A701X_Point},
        {"CX7-A701CR", CX7_A701X_Point},
        {"CX7-A901S", CX7_A901X_Point},
        {"CX7-A901SR", CX7_A901X_Point},
        {"CX7-A901C", CX7_A901X_Point},
        {"CX7-A901CR", CX7_A901X_Point}
    };


    public:
    EpsonRBPickPlacePoint();
    ~EpsonRBPickPlacePoint();
    PickPlacePoint LoadPickPlacePoint(const string& rb_model) {return AllRBPickPalcePoint[rb_model];};
    bool CanLoadPickPlacePoint(const string& rb_model);

};