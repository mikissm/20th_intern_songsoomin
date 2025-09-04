#pragma once
#include <iostream>
using namespace std;

struct Point {
    int x;
    int y;
};

struct Arr {
    int range_minimum;
    int range_maximum;
    Point* arr;
    int size;

    Arr(int n, int min, int max);
    ~Arr();
};

class Dot {
private:
    int num;
    Arr points;
    float max_dist;
    float min_dist;
    pair<int,int> min_pair;
    pair<int,int> max_pair;

public:
    Dot(int n, int min, int max);
    ~Dot();
    void create_rand_dot();
    void print();
    double distance(int i, int j);
    void calculation_length();
};

double mySqrt(double x);

void inputInt(const char* prompt, int* value);