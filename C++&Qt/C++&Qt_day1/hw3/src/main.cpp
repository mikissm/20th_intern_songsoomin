#include "hw2.hpp"
#include <iostream>
using namespace std;

int main() {
    Player player(0, 0);
    Monster monster(5, 4, 50);

    while (1) {
        if (player.HP == 0) {
            cout << "Monster Die!!" << endl;
            return 0;
        }

        char ans;
        cout << "Type Command(A/U/D/R/L/S)" << endl;
        cin >> ans;

        char c;
        while (cin.get(c) && c != '\n');

        switch (ans) {
        case 'A':
            if (player.MP == 0) {
                cout << "MP 부족!" << endl;
                return 0;
            }
            player.Attack(monster);
            break;

        case 'U': player.Y_move(1); break;
        case 'D': player.Y_move(-1); break;
        case 'R': player.X_move(1); break;
        case 'L': player.X_move(-1); break;
        case 'S': player.Show_Status(); break;
        default: cout << "command error" << endl; break;
        }
    }
}
