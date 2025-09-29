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
#include "epson_robot_demo/epson_robot_pickplace_point.hpp"

EpsonRBPickPlacePoint::EpsonRBPickPlacePoint(){
    ;
}

EpsonRBPickPlacePoint::~EpsonRBPickPlacePoint(){
    ;
}

bool  EpsonRBPickPlacePoint::CanLoadPickPlacePoint(const string& rb_model)
{
    map<string,PickPlacePoint>::iterator temp_rb_model;
    temp_rb_model=AllRBPickPalcePoint.find(rb_model);
    if(temp_rb_model == AllRBPickPalcePoint.end()) return false;

    return true;
}
