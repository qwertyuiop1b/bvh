#include <random>


namespace math {

    template<typename T>
    T random(T min, T max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<T> dis(min, max);
        return dis(gen);
    }
}