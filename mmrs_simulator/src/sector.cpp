// The MIT License (MIT)

// Copyright (c) 2019 Piotr Dulewicz

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

/**
 * @file sector.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-06-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "../include/mmrs_simulator/sector.h"

using namespace mmrs;

Sector::Sector(double endpoint, SectorSet colliding) : endpoint_{endpoint},
                                                               colliding_sectors_{colliding}
{
}

bool Sector::IsColliding(std::pair<int, int> index) const
{
  auto search = colliding_sectors_.find(index);
  return search != colliding_sectors_.end();
}


void mmrs::to_json(json &j, const Sector &s)
{
  j["Endpoint"] = s.endpoint_;
  j["Colliding"] = json::array();
  for(const auto& x : s.colliding_sectors_)
  {
    j["Colliding"].push_back(json::array({x.first, x.second}));
  }
}
void mmrs::from_json(const json &j, Sector &s)
{


}