#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        // Perlin noise setup
        // Initialize the permutation vector with values 0 to 255
        permutationVector.resize(256);
        std::iota(permutationVector.begin(), permutationVector.end(), 0);

        // Shuffle the vector using the seed
        unsigned int seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::default_random_engine engine(seed);
        std::shuffle(permutationVector.begin(), permutationVector.end(), engine);

        // Duplicate the vector to avoid bounds checking
        permutationVector.insert(permutationVector.end(), permutationVector.begin(), permutationVector.end());

        //Publisher setup
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        auto message = nav_msgs::msg::OccupancyGrid();
        message.header.frame_id = "map";
        message.info.resolution = 1.0;
        message.info.width = 100;
        message.info.height = 100;

        for(size_t r = 0; r < message.info.width; r++) {
            for(size_t c = 0; c < message.info.height; c++) {
                double noise_value = fractalBrownianMotion(c, r, 8);
                double scaled_value = (noise_value + 1.0) * 50.0; // Map to [0, 100]
                if(scaled_value > 45) {
                    message.data.push_back(100);
                } else {
                    message.data.push_back(0);
                }
            } 
        }

        RCLCPP_INFO(this->get_logger(), "Publishing map");
        this->publisher_->publish(message);
    }

private:
    std::vector<int> permutationVector;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

    double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    double fade(double t) {
        return ((6 * t - 15) * t + 10) * t * t * t;
    }

    double grad(int hash, double x, double y) {
        const int h = hash & 3;
        const double u = h < 2 ? x : y;
        const double v = h < 2 ? y : x;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    // Implementation based on https://rtouti.github.io/graphics/perlin-noise-algorithm
    double perlinNoise(double x, double y) {
        // Find grid cell coordinates
        const int X = static_cast<int>(floor(x)) & 255;
        const int Y = static_cast<int>(floor(y)) & 255;

        // Find fractional coordinates
        const double xf = x - floor(x);
        const double yf = y - floor(y);

        // Apply fade curve
        const double u = fade(xf);
        const double v = fade(yf);

        // Hash coordinates of the 4 corners
        const int aa = permutationVector[permutationVector[X] + Y];
        const int ab = permutationVector[permutationVector[X] + Y + 1];
        const int ba = permutationVector[permutationVector[X + 1] + Y];
        const int bb = permutationVector[permutationVector[X + 1] + Y + 1];

        // Blend the results from the 4 corners
        const double x1 = lerp(grad(aa, xf, yf), grad(ba, xf - 1, yf), u);
        const double x2 = lerp(grad(ab, xf, yf - 1), grad(bb, xf - 1, yf - 1), u);

        return lerp(x1, x2, v);
    }

    double fractalBrownianMotion(double x, double y, int numOctaves) {
        double total = 0.0;
        double amplitude = 1.0;
        double frequency = 0.05; 
        double maxAmplitude = 0.0;

        for (int i = 0; i < numOctaves; i++) {
            total += perlinNoise(x * frequency, y * frequency) * amplitude;

            maxAmplitude += amplitude; 
            amplitude *= 0.5;
            frequency *= 2.0;
        }

        return total / maxAmplitude; // Normalize to [-1, 1] range
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublisher>());
    rclcpp::shutdown();
    return 0;
}