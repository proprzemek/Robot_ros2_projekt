#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

// Struktura opisująca pojedynczy krok (kratkę) na mapie
struct AStarNode {
    int x, y;
    double g, f;
    std::shared_ptr<AStarNode> parent;

    AStarNode(int x, int y, double g, double h, std::shared_ptr<AStarNode> p = nullptr)
        : x(x), y(y), g(g), f(g + h), parent(p) {}
};

// Komparator dla algorytmu - zawsze wybieramy kratkę z najmniejszym kosztem F
struct CompareNode {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f > b->f;
    }
};

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        RCLCPP_INFO(this->get_logger(), "Faza 3: Uruchamiam sztuczna inteligencje (A*)...");
        timer_ = this->create_wall_timer(
            100ms, std::bind(&NavigationNode::process_image, this));
    }

private:
    void process_image() {
        // 1. Rysujemy naszą bazową mapę
        cv::Mat grid = cv::Mat::zeros(500, 500, CV_8UC3);
        cv::rectangle(grid, cv::Point(50, 50), cv::Point(450, 450), cv::Scalar(255, 255, 255), -1);
        cv::circle(grid, cv::Point(250, 250), 40, cv::Scalar(0, 0, 0), -1);

        // 2. Definiujemy Start i Cel
        cv::Point start(100, 100);
        cv::Point goal(400, 400);

        // 3. Uruchamiamy algorytm A*
        std::vector<cv::Point> path = find_path_astar(start, goal, grid);

        // 4. Rysujemy wyznaczoną ścieżkę (Niebieska linia)
        for (size_t i = 1; i < path.size(); ++i) {
            cv::line(grid, path[i - 1], path[i], cv::Scalar(255, 0, 0), 3);
        }

        // 5. Rysujemy punkty (żeby były nad linią)
        cv::circle(grid, start, 6, cv::Scalar(0, 255, 0), -1);
        cv::circle(grid, goal, 6, cv::Scalar(0, 0, 255), -1);

        cv::imshow("Podglad - A* Pathfinding", grid);
        cv::waitKey(1);
    }

    // --- GŁÓWNY ALGORYTM A* ---
    std::vector<cv::Point> find_path_astar(cv::Point start, cv::Point goal, const cv::Mat& map) {
        int step = 10; // Krok siatki (10 pikseli)
        
        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareNode> open_set;
        bool closed_set[500][500] = {false}; // Tablica odwiedzonych miejsc

        // Dodajemy punkt startowy
        open_set.push(std::make_shared<AStarNode>(start.x, start.y, 0.0, heuristic(start, goal)));

        // Możliwe ruchy (góra, dół, lewo, prawo + skosy)
        int dx[] = {-step, step, 0, 0, -step, -step, step, step};
        int dy[] = {0, 0, -step, step, -step, step, -step, step};

        while (!open_set.empty()) {
            auto current = open_set.top();
            open_set.pop();

            // Jeśli jesteśmy przy celu (w promieniu jednego kroku)
            if (std::abs(current->x - goal.x) <= step && std::abs(current->y - goal.y) <= step) {
                std::vector<cv::Point> path;
                auto temp = current;
                while (temp != nullptr) {
                    path.push_back(cv::Point(temp->x, temp->y));
                    temp = temp->parent;
                }
                return path; // Zwracamy gotową trasę
            }

            if (closed_set[current->x][current->y]) continue;
            closed_set[current->x][current->y] = true;

            // Sprawdzamy sąsiadów
            for (int i = 0; i < 8; ++i) {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];

                // Sprawdzamy, czy nie wychodzimy za mapę i czy nie uderzamy w czarną przeszkodę
                if (nx >= 0 && nx < 500 && ny >= 0 && ny < 500 && !closed_set[nx][ny]) {
                    cv::Vec3b color = map.at<cv::Vec3b>(ny, nx);
                    if (color != cv::Vec3b(0, 0, 0)) { // Jeśli to nie jest czarny mur
                        double cost = (i < 4) ? 1.0 : 1.414; // Skosy są "droższe"
                        open_set.push(std::make_shared<AStarNode>(
                            nx, ny, current->g + cost, heuristic(cv::Point(nx, ny), goal), current));
                    }
                }
            }
        }
        return {}; // Zwraca pusto, jeśli nie ma drogi
    }

    // Funkcja "węchu" (odległość w linii prostej do celu)
    double heuristic(cv::Point a, cv::Point b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)) / 10.0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}