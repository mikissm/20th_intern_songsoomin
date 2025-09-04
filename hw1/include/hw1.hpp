

namespace hw1 {
    class calculation {
    private:
        int sum;
        float ave;
        int max;
        int min;
        int size;
        int *a;

    public:
        calculation(int size);
        ~calculation();

        void input();
        void compute();
        void display();
    };
}

void inputInt(const char* prompt, int* value);