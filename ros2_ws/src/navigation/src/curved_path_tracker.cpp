#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>

struct PathCommand {
    double x, y, radius, angle; // x[m], y[m], radius[m], angle[deg]
};

class CurvedPathPublisher : public rclcpp::Node {
public:
    CurvedPathPublisher() : Node("curved_path_publisher") {
        // ===== 參數 =====
        int timer_ms = this->declare_parameter<int>("timer_ms", 50);
        dt_ = timer_ms / 1000.0;

        v_straight_ = this->declare_parameter<double>("v_straight", 0.25); // 直線段上限 m/s
        a_lat_max_  = this->declare_parameter<double>("a_lat_max", 0.80);  // 最大向心加速度 m/s^2
        v_max_      = this->declare_parameter<double>("v_max", 0.30);      // 線速度硬上限 m/s
        omega_max_  = this->declare_parameter<double>("omega_max", 2.0);   // 角速度硬上限 rad/s

        k_long_     = this->declare_parameter<double>("k_long",  0.5);     // 進度誤差 -> 切向速度
        k_lat_      = this->declare_parameter<double>("k_lat",   1.0);     // 橫向誤差 -> 法向速度
        k_theta_    = this->declare_parameter<double>("k_theta", 2.0);     // 朝向誤差 -> 角速度

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 回授姿態（可選）：如果你有發布 pose_feedback，就能閉迴路
        sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose_feedback", 10,
            std::bind(&CurvedPathPublisher::poseCallback, this, std::placeholders::_1));

        path_ = readPath("src/navigation/waypoints/center_path.csv");
        if (path_.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid path found.");
            return;
        }

        simulateAndWrite(path_, "path.dat");         // 寫入 path.dat 給 Gnuplot
        plotWithGnuplot("path.dat");                 // 呼叫 Gnuplot 畫圖

        i_ = 1;
        step_ = 0;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_ms),
            std::bind(&CurvedPathPublisher::publishTwist, this));
    }

private:
    // ===== ROS =====
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ===== 路徑/狀態 =====
    std::vector<PathCommand> path_;
    size_t i_ = 1;   // 當前段：p[i-1] -> p[i]
    int step_ = 0;
    int max_step_ = 50;

    double prev_x_=0.0, prev_y_=0.0, prev_theta_=0.0;

    // 回授姿態（世界座標）
    bool have_pose_ = false;
    double x_=0.0, y_=0.0, yaw_=0.0;

    // 參數/增益
    double dt_ = 0.05;
    double v_straight_=0.25, a_lat_max_=0.8, v_max_=0.30, omega_max_=2.0;
    double k_long_=0.5, k_lat_=1.0, k_theta_=2.0;

    // ===== 工具 =====
    static double wrap_pi(double a){
        while(a >  M_PI) a -= 2*M_PI;
        while(a < -M_PI) a += 2*M_PI;
        return a;
    }
    static double yaw_from_quat(const geometry_msgs::msg::Quaternion& q){
        // 若你暫時把 yaw 塞在 orientation.z，且 quat 無效，可改用 return q.z;
        double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
        double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr p){
        x_ = p->position.x;
        y_ = p->position.y;
        if (p->orientation.w == 0.0 && p->orientation.x == 0.0 && p->orientation.y == 0.0) {
            yaw_ = p->orientation.z;  // 兼容你之前把 yaw 放在 z
        } else {
            yaw_ = yaw_from_quat(p->orientation);
        }
        have_pose_ = true;
    }

    std::vector<PathCommand> readPath(const std::string& filename) {
        std::vector<PathCommand> path;
        std::ifstream file(filename);
        std::string line;
        int lineno=0;

        while (std::getline(file, line)) {
            ++lineno;
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str, r_str, a_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, r_str, ',');
            std::getline(ss, a_str, ','); // 四欄格式：x,y,r,angle

            try {
                double x = std::stod(x_str) / 1000.0; // mm -> m
                double y = std::stod(y_str) / 1000.0;
                double r = std::stod(r_str) / 1000.0;
                double a = std::stod(a_str);          // deg
                // 允許 r<0 或 angle≈0 代表直線
                if (r < 0.0 || std::abs(a) < 1e-6) { r = 0.0; a = 0.0; }
                path.push_back({x, y, r, a});
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Invalid line %d: %s", lineno, line.c_str());
            }
        }
        return path;
    }

    // 計算段的參考點與參考朝向（直線/圓弧）
    void refOnSegment(const PathCommand& p1, const PathCommand& p2,
                      int step, int max_step,
                      double& xi, double& yi, double& theta_ref,
                      double& kappa, double& seg_len)
    {
        if (std::abs(p1.angle) < 1e-6) {
            // 直線
            double t = static_cast<double>(step) / std::max(1, max_step);
            xi = p1.x + (p2.x - p1.x) * t;
            yi = p1.y + (p2.y - p1.y) * t;
            theta_ref = std::atan2(p2.y - p1.y, p2.x - p1.x);
            kappa = 0.0;
            seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
        } else {
            // 圓弧
            double theta_rad = p1.angle * M_PI / 180.0; // >0 CCW, <0 CW
            double dx = p2.x - p1.x, dy = p2.y - p1.y;
            double chord = std::hypot(dx, dy);
            double r = std::abs(p1.radius);
            // 守護：半徑太小會 sqrt 負，強制 >= chord/2
            double min_r = 0.5*chord + 1e-6;
            if (r < min_r) r = min_r;

            double mid_x = (p1.x + p2.x) * 0.5;
            double mid_y = (p1.y + p2.y) * 0.5;
            double dir_x = -dy / chord;
            double dir_y =  dx / chord;
            double h2 = std::max(0.0, r*r - (chord*chord)/4.0);
            double h  = std::sqrt(h2);
            double cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
            double cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);

            double ang1 = std::atan2(p1.y - cy, p1.x - cx);
            double ang2 = std::atan2(p2.y - cy, p2.x - cx);
            if (theta_rad > 0 && ang2 < ang1) ang2 += 2*M_PI;
            if (theta_rad < 0 && ang2 > ang1) ang2 -= 2*M_PI;
            double d_ang = ang2 - ang1;

            double t = static_cast<double>(step) / std::max(1, max_step);
            double ang = ang1 + d_ang * t;
            xi = cx + r * std::cos(ang);
            yi = cy + r * std::sin(ang);

            theta_ref = ang + std::copysign(M_PI_2, theta_rad);
            kappa = std::copysign(1.0, theta_rad) / r;
            seg_len = std::abs(r * d_ang);
        }
    }

    // 依曲率決定段內目標線速
    double v_target_from_kappa(double kappa){
        if (std::abs(kappa) < 1e-9) return v_straight_;
        double v_turn = std::sqrt(a_lat_max_ / std::max(1e-9, std::abs(kappa)));
        return std::min(v_straight_, v_turn);
    }

    void publishTwist() {
        if (i_ >= path_.size()) return;

        const auto& p1 = path_[i_ - 1];
        const auto& p2 = path_[i_];

        double xi, yi, theta_ref, kappa, seg_len;
        refOnSegment(p1, p2, std::max(0, step_), std::max(1, max_step_), xi, yi, theta_ref, kappa, seg_len);

        // 第一次進段：依速度目標決定 max_step_
        if (step_ == 0) {
            double v_des = v_target_from_kappa(kappa);
            max_step_ = std::max(1, (int)std::ceil(seg_len / (v_des * dt_)));

            // 避免第一拍速度尖峰：把 prev_* 對齊參考點後直接 return
            prev_x_ = xi; prev_y_ = yi; prev_theta_ = theta_ref;
            geometry_msgs::msg::Twist zero; pub_->publish(zero);
            step_ = 1;
            return;
        }

        // ===== 前饋（世界座標）：延用你原本差分法 =====
        double vx_ff = (xi - prev_x_) / dt_;
        double vy_ff = (yi - prev_y_) / dt_;
        double omega_ff = (theta_ref - prev_theta_) / dt_;
        omega_ff = wrap_pi(omega_ff * dt_) / dt_; // wrap 一下避免跨 ±π 跳變

        // ===== 回授（可選）：有 pose 才加，維持原路徑 =====
        double vx_w = vx_ff;
        double vy_w = vy_ff;
        double omega = omega_ff;

        if (have_pose_) {
            // 切向/法向單位向量（世界座標）
            double ct = std::cos(theta_ref), st = std::sin(theta_ref);
            double nx = -st, ny = ct;

            // 位置誤差（世界座標）
            double ex = x_ - xi;
            double ey = y_ - yi;

            // 進度/橫向/朝向誤差
            double e_long  =  ex * ct + ey * st;
            double e_lat   =  ex * nx + ey * ny;
            double e_theta =  wrap_pi(yaw_ - theta_ref);

            // 在世界座標做前饋+回授（切向加速、法向拉回、角度拉回）
            vx_w = (v_target_from_kappa(kappa) + k_long_ * e_long) * ct
                   - (k_lat_ * e_lat) * st;
            vy_w = (v_target_from_kappa(kappa) + k_long_ * e_long) * st
                   + (k_lat_ * e_lat) * ct;

            // 角速度 = 前饋曲率 v + 朝向誤差回授
            omega = kappa * v_target_from_kappa(kappa) - k_theta_ * e_theta;
        }

        // ===== 速度上限（同比例縮放，保 v/ω 幾何） =====
        double vmag = std::hypot(vx_w, vy_w);
        double scale = 1.0;
        if (vmag > v_max_)            scale = std::min(scale, v_max_ / (vmag + 1e-9));
        if (std::abs(omega) > omega_max_) scale = std::min(scale, omega_max_ / (std::abs(omega) + 1e-9));
        vx_w *= scale; vy_w *= scale; omega *= scale;

        // ===== 發佈（直接發世界座標 vx, vy。如果你的底盤需要 base_link，請在此旋轉到機器人座標）=====
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = vx_w;
        twist.linear.y  = vy_w;
        twist.angular.z = omega;
        pub_->publish(twist);

        // ===== 更新狀態 =====
        prev_x_ = xi; prev_y_ = yi; prev_theta_ = theta_ref;

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

            if (std::abs(p1.angle) < 1e-6) {
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
                double min_r = 0.5*chord + 1e-6;
                if (r < min_r) r = min_r;

                double mid_x = (p1.x + p2.x) / 2.0;
                double mid_y = (p1.y + p2.y) / 2.0;
                double dir_x = -dy / chord;
                double dir_y = dx / chord;
                double h = std::sqrt(std::max(0.0, r*r - (chord*chord)/4.0));
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
