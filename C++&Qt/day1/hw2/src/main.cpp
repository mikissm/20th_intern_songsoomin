#include "hw2.hpp"
#include <iostream>
using namespace std;

int main() {
    int max, min, num;
    
    inputInt("좌표의 최솟값을 입력하세요 : ", &min);
    inputInt("좌표의 최댓값을 입력하세요 : ", &max);
    inputInt("생성할 점의 개수를 입력하세요 : ", &num);

    if ((max - min + 1) * (max - min + 1) < num) {
        cout << "점의 갯수가 너무 많습니다." << endl;
        return 0;
    }
    if (num < 2) {
        cout << "점의 갯수가 너무 적습니다." << endl;
        return 0;
    }
    if (max < min) {
        cout << "최솟값과 최댓값이 옳바르지 않습니다." << endl;
        return 0;
    }

    Dot d(num, min, max);
    d.create_rand_dot();
    d.print();
    d.calculation_length();

    return 0;
}
