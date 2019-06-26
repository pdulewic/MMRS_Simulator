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
 * @file sector.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-06-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <unordered_set>
#include <utility>
#include <functional>

#include "../../nlohmann/json.hpp"

using nlohmann::json;

namespace mmrs
{

struct PairIntHash
{
  inline std::size_t operator()(const std::pair<int, int> &p) const
  {
    return std::hash<int>{}(p.first) ^ std::hash<int>{}(p.second);
  }
};

class Sector
{
  using SectorSet = std::unordered_set<std::pair<int, int>, PairIntHash>;
  double endpoint_;
  SectorSet colliding_sectors_;

public:
  Sector(double endpoint = -1.0, SectorSet colliding = SectorSet());
  bool IsColliding(std::pair<int, int> index) const;
  double GetEndpoint() const { return endpoint_; }

  friend void mmrs::to_json(json &j, const Sector &s);
  friend void mmrs::from_json(const json &j, Sector &s);
};

// special functions descriebed in
// https://github.com/nlohmann/json#arbitrary-types-conversions
void to_json(json &j, const Sector &s);
void from_json(const json &j, Sector &s);

} // namespace mmrs
