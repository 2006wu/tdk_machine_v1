#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>

struct PathCommand {
    double x, y, radius, angle;
};

class CurvedPathPublisher : public rclcpp::Node {
public:
    CurvedPathPublisher() : Node("curved_path_publisher") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        path_ = readPath("src/navigation/waypoints/center_path.csv");
        if (path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid path found.");
            return;
        }

        simulateAndWrite(path_, "path.dat");         // 寫入 path.dat 給 Gnuplot
        plotWithGnuplot("path.dat");                 // 呼叫 Gnuplot 畫圖

        i_ = 1;
        step_ = 0;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CurvedPathPublisher::publishTwist, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PathCommand> path_;
    size_t i_ = 1;
    int step_ = 0;
    int max_step_ = 50;

    double prev_x_, prev_y_, prev_theta_;

    std::vector<PathCommand> readPath(const std::string& filename) {
        std::vector<PathCommand> path;
        std::ifstream file(filename);
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str, r_str, _, a_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, r_str, ',');
            //std::getline(ss, _, ',');  // 跳過方向欄位
            std::getline(ss, a_str, ',');

            try {
                double x = std::stod(x_str) / 1000.0;
                double y = std::stod(y_str) / 1000.0;
                double r = std::stod(r_str) / 1000.0;
                double a = std::stod(a_str);
                path.push_back({x, y, r, a});
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Invalid line: %s", line.c_str());
            }
        }
        return path;
    }

    void publishTwist() {
        if (i_ >= path_.size()) return;

        const auto& p1 = path_[i_ - 1];
        const auto& p2 = path_[i_];

        double xi, yi, theta;

        if (std::abs(p1.angle) < 1e-3) {
            double t = static_cast<double>(step_) / max_step_;
            xi = p1.x + (p2.x - p1.x) * t;
            yi = p1.y + (p2.y - p1.y) * t;
            theta = std::atan2(p2.y - p1.y, p2.x - p1.x);
        } else {
            double theta_rad = p1.angle * M_PI / 180.0;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double chord = std::hypot(dx, dy);
            double r = std::abs(p1.radius);
            double mid_x = (p1.x + p2.x) / 2.0;
            double mid_y = (p1.y + p2.y) / 2.0;
            double dir_x = -dy / chord;
            double dir_y = dx / chord;
            double h = std::sqrt(r * r - (chord * chord) / 4.0);
            double cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
            double cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);

            double start_angle = std::atan2(p1.y - cy, p1.x - cx);
            double end_angle = std::atan2(p2.y - cy, p2.x - cx);
            if (theta_rad > 0 && end_angle < start_angle) end_angle += 2 * M_PI;
            if (theta_rad < 0 && end_angle > start_angle) end_angle -= 2 * M_PI;
            double delta = end_angle - start_angle;
            double t = static_cast<double>(step_) / max_step_;
            double angle = start_angle + delta * t;
            xi = cx + r * std::cos(angle);
            yi = cy + r * std::sin(angle);
            theta = angle + M_PI_2 * std::copysign(1.0, theta_rad);
        }

        // 在 publishTwist() 中 step_ == 0 時設定 max_step_
        if (step_ == 0) {
            double path_length;
            if (std::abs(p1.angle) < 1e-3) {
                path_length = std::hypot(p2.x - p1.x, p2.y - p1.y);  // 直線距離
            } else {
                path_length = std::abs(p1.radius) * std::abs(p1.angle) * M_PI / 180.0;  // 弧長 = rθ
            }

            /// 以 0.2 m/s 的速度和 50 ms 的控制間隔計算 max_step_ ///
            double desired_speed = 0.2;  // m/s
            double dt = 0.05;            // 控制間隔
            max_step_ = std::ceil(path_length / (desired_speed * dt));
        }

        double dt = 0.05;
        double vx = (xi - prev_x_) / dt;
        double vy = (yi - prev_y_) / dt;
        double omega = (theta - prev_theta_) / dt;
        // omega = limit(omega, 1.0);  // 例如 max_omega = 1.0 rad/s

        // give the message to STM
        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.angular.z = omega;
        pub_->publish(twist);

        prev_x_ = xi;
        prev_y_ = yi;
        prev_theta_ = theta;

        step_++;
        if (step_ > max_step_) {
            step_ = 0;
            i_++;
        }
    }

    void simulateAndWrite(const std::vector<PathCommand>& path, const std::string& datafile) {
        std::ofstream out(datafile);
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& p1 = path[i - 1];
            const auto& p2 = path[i];

            if (std::abs(p1.angle) < 1e-3) {
                for (int s = 0; s <= 50; ++s) {
                    double t = static_cast<double>(s) / 50;
                    double xi = p1.x + (p2.x - p1.x) * t;
                    double yi = p1.y + (p2.y - p1.y) * t;
                    out << xi << " " << yi << "\n";
                }
            } else {
                double theta_rad = p1.angle * M_PI / 180.0;
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double chord = std::hypot(dx, dy);
                double r = std::abs(p1.radius);
                double mid_x = (p1.x + p2.x) / 2.0;
                double mid_y = (p1.y + p2.y) / 2.0;
                double dir_x = -dy / chord;
                double dir_y = dx / chord;
                double h = std::sqrt(r*r - (chord*chord)/4.0);
                double cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
                double cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);
                double start_angle = std::atan2(p1.y - cy, p1.x - cx);
                double end_angle   = std::atan2(p2.y - cy, p2.x - cx);
                if (theta_rad > 0 && end_angle < start_angle) end_angle += 2 * M_PI;
                if (theta_rad < 0 && end_angle > start_angle) end_angle -= 2 * M_PI;
                double delta_angle = end_angle - start_angle;

                for (int s = 0; s <= 50; ++s) {
                    double t = static_cast<double>(s) / 50;
                    double angle = start_angle + delta_angle * t;
                    double xi = cx + r * std::cos(angle);
                    double yi = cy + r * std::sin(angle);
                    out << xi << " " << yi << "\n";
                }
            }
        }
        out.close();
    }

    void plotWithGnuplot(const std::string& datafile) {
        std::ofstream plt("plot_path.plt");
        plt << "set title 'Robot Path'\n";
        plt << "set xlabel 'X (m)'\n";
        plt << "set ylabel 'Y (m)'\n";
        plt << "set grid\n";
        plt << "set size ratio -1\n";
        plt << "plot '" << datafile << "' with linespoints title 'Path'\n";
        plt << "pause 5\n";
        plt.close();
        system("gnuplot plot_path.plt");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurvedPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
