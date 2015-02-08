#ifndef RunningAverage_h
#define RunningAverage_h
//
//	FILE: RunningAverage.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// PURPOSE: RunningAverage library for Arduino
// VERSION: 0.1.01
// 	URL: http://arduino.cc/playground/Main/RunningAverage
// HISTORY: See RunningAverage.cpp
//
// Released to the public domain
//

#include <stdio.h>

class RunningAverage
{
    public:
    RunningAverage(int);
    ~RunningAverage();
    void clr();
    void add(float);
    float avg();
    float avg2();
    float avg3();

protected:
    int _size;
    int _cnt;
    int _idx;
    float _sum;
    float * _ar;
};

#endif
