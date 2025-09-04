#include <iostream>
#include "hw1.hpp"
using namespace std;

int main() {
    int s;
    inputInt("몇 개의 원소를 할당하시겠습니까? : ", &s);
    if (s < 1) {
        cout << "입력한 숫자가 너무 작습니다." << endl;
        return 0;
    }
    hw1::calculation calc(s);
    calc.input();
    calc.compute();
    calc.display();
    return 0;
}