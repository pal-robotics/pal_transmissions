/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef RANDOM_GENERATOR_UTILS_H
#define RANDOM_GENERATOR_UTILS_H

#include <cstdlib>
#include <ctime>
#include <vector>

using std::vector;

/// \brief Generator of pseudo-random double in the range [min_val, max_val].
// NOTE: Based on example code available at:
// http://stackoverflow.com/questions/2860673/initializing-a-c-vector-to-random-values-fast
struct RandomDoubleGenerator
{
public:
  RandomDoubleGenerator(double min_val, double max_val)
    : min_val_(min_val),
      max_val_(max_val) {srand(time(NULL));}
  double operator()()
  {
    const double range = max_val_ - min_val_;
    return rand() / static_cast<double>(RAND_MAX) * range + min_val_;
  }
private:
  double min_val_;
  double max_val_;
};

/// \brief Generator of a vector of pseudo-random doubles.
vector<double> randomVector(const vector<double>::size_type size, RandomDoubleGenerator& generator)
{
  vector<double> out;
  out.reserve(size);
  for (vector<double>::size_type i = 0; i < size; ++i) {out.push_back(generator());}
  return out;
}

#endif // RANDOM_GENERATOR_UTILS_H