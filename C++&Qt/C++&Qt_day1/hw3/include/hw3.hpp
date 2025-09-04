#pragma once
#include <iostream>
using namespace std;

class Monster {
public:
    int HP, x, y;
    Monster();
    Monster(int x, int y, int HP);
    int Be_Attacked();
};

class Player {
public:
    int HP, MP, x, y;
    Player();
    Player(int x, int y);
    void Attack(Monster &target);
    void Show_Status();
    void X_move(int move);
    void Y_move(int move);
};
