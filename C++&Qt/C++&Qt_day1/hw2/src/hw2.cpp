#include "hw2.hpp"
#include <random>

Arr::Arr(int n, int min, int max) {
    range_minimum = min;
    range_maximum = max;
    size = n;
    arr = new Point[size];
}

Arr::~Arr() {
    delete[] arr;
}

Dot::Dot(int n, int min, int max)
    : num(n), points(n, min, max), max_dist(0), min_dist(1e9) {}

Dot::~Dot() {}

void Dot::create_rand_dot() {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dist(points.range_minimum, points.range_maximum);

    int placed = 0;
    while (placed < num) {
        int x = dist(gen);
        int y = dist(gen);
        bool duplicate = false;
        for (int j = 0; j < placed; j++) {
            if (points.arr[j].x == x && points.arr[j].y == y) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            points.arr[placed].x = x;
            points.arr[placed].y = y;
            placed++;
        }
    }
}

void Dot::print() {
    for (int i = 0; i < num; i++) {
        cout << i << "번 점: (" << points.arr[i].x << ", " << points.arr[i].y << ")\n";
    }
}

double Dot::distance(int i, int j) {
    int dx = points.arr[i].x - points.arr[j].x;
    int dy = points.arr[i].y - points.arr[j].y;
    return mySqrt((double)(dx*dx + dy*dy));
}

void Dot::calculation_length() {
    for (int i = 0; i < num; i++) {
        for (int j = i+1; j < num; j++) {
            double d = distance(i, j);
            if (d > max_dist) {
                max_dist = d;
                max_pair = {i, j};
            }
            if (d < min_dist) {
                min_dist = d;
                min_pair = {i, j};
            }
        }
    }

    cout << "\n=== 거리 계산 결과 ===\n";
    cout << "최소 거리: " << min_dist << " ("
         << min_pair.first << "번 점 " << "(" << points.arr[min_pair.first].x << ", " << points.arr[min_pair.first].y << ")"
         << " ↔ "
         << min_pair.second << "번 점 " << "(" << points.arr[min_pair.second].x << ", " << points.arr[min_pair.second].y << "))\n";

    cout << "최대 거리: " << max_dist << " ("
         << max_pair.first << "번 점 " << "(" << points.arr[max_pair.first].x << ", " << points.arr[max_pair.first].y << ")"
         << " ↔ "
         << max_pair.second << "번 점 " << "(" << points.arr[max_pair.second].x << ", " << points.arr[max_pair.second].y << "))\n";
}

double mySqrt(double x) {
    if (x == 0) return 0;
    double guess = x;
    for (int i = 0; i < 20; i++) {
        guess = 0.5 * (guess + x / guess);
    }
    return guess;
}

void inputInt(const char* prompt, int* value) {
    while (true) {
        cout << prompt;
        double temp;
        cin >> temp;

        if (cin.fail()) {
            cin.clear();
            char c;
            while (cin.get(c) && c != '\n');
            cout << "잘못된 입력입니다. 정수를 입력해주세요." << endl;
            continue;
        }

        if (temp != static_cast<int>(temp)) {
            cout << "정수를 입력해주세요." << endl;
            continue;
        }

        *value = static_cast<int>(temp);
        break;
    }
}
