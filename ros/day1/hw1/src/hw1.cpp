#include "hw1.hpp"
#include <iostream>
using namespace std;

namespace hw1 {
    calculation::calculation(int sz) : size(sz) {
        a = new int[size];
        sum = 0;
        ave = 0;
        max = 0;
        min = 0;
    }

    calculation::~calculation() {
        delete[] a;
    }

    void calculation::input() {
        for(int i=0; i<size; i++) {
            inputInt("정수형 데이터 입력:", &a[i]);
        }
    }

    void calculation::compute() {
        sum = 0;
        max = a[0];
        min = a[0];
        for(int i=0; i<size; i++) {
            sum += a[i];
            if(a[i] > max) max = a[i];
            if(a[i] < min) min = a[i];
        }
        ave = (float)sum / size;
    }

    void calculation::display() {
        cout << "최댓값: " << max << endl;
        cout << "최솟값: " << min << endl;
        cout << "전체합: " << sum << endl;
        cout << "평 균: " << ave << endl;
    }
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

        // 소수 체크
        if (temp != static_cast<int>(temp)) {
            cout << "정수를 입력해주세요." << endl;
            continue;
        }

        *value = static_cast<int>(temp);
        break;
    }
}