#include "hw2.hpp"
#include <iostream>
using namespace std;

Player::Player() {
    HP = 50;
    MP = 10;
}

Player::Player(int x_val, int y_val) {
    HP = 50;
    MP = 10;
    x = x_val;
    y = y_val;
}

Monster::Monster() {
    HP = 50;
    x = 0;
    y = 0;
}

Monster::Monster(int x_val, int y_val, int hp_val) {
    x = x_val;
    y = y_val;
    HP = hp_val;
}

// Player 멤버 함수
void Player::Attack(Monster &target) {
    MP--;
    if ((target.x == x) && (target.y == y)) {
        cout << "공격 성공!" << endl;
        HP = target.Be_Attacked();
    }
    else {
        cout << "공격 실패!" << endl;
    }
}

void Player::Show_Status() {
    cout << "HP:" << HP << endl;
    cout << "MP:" << MP << endl;
    cout << "Position:" << x << ',' << y << endl;
}

void Player::X_move(int move) {
    x += move;
    cout << "X Position " << move << " moved!" << endl;
}

void Player::Y_move(int move) {
    y += move;
    cout << "Y Position " << move << " moved!" << endl;
}

// Monster 멤버 함수
int Monster::Be_Attacked() {
    HP -= 10;
    cout << "남은 체력:" << HP << endl;
    return HP;
}
