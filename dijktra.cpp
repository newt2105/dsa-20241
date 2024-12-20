#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
using namespace std;

typedef pair<double, int> ii; // {trọng số cạnh, đỉnh}
const double INF = numeric_limits<double>::infinity();

class Graph {
private:
    vector<vector<ii>> adjList;    // Danh sách kề của đồ thị
    vector<double> dist;          // Khoảng cách từ nguồn đến các đỉnh
    vector<int> pre;              // Đỉnh trước đó trên đường đi
    vector<pair<int, int>> points; // Tọa độ các điểm

public:
    Graph(int n, const vector<pair<int, int>>& coordinates) {
        adjList.resize(n);
        dist.resize(n, INF);
        pre.resize(n, -1);
        points = coordinates;
    }

    // Hàm thêm liên kết vào đồ thị
    void addEdge(int u, int v) {
        double weight = calculateDistance(points[u], points[v]);
        adjList[u].push_back({weight, v});
        adjList[v].push_back({weight, u}); // Đồ thị hai chiều
    }

    // Hàm tính khoảng cách Euclidean giữa 2 điểm
    double calculateDistance(pair<int, int> a, pair<int, int> b) {
        return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
    }

    // Thuật toán Dijkstra
    void dijkstra(int source) {
        priority_queue<ii, vector<ii>, greater<ii>> pq; // Hàng đợi ưu tiên (min-heap)
        dist.assign(adjList.size(), INF);
        pre.assign(adjList.size(), -1);

        dist[source] = 0.0;
        pq.push({0.0, source});

        while (!pq.empty()) {
            double d_u = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            if (d_u > dist[u]) continue; // Nếu khoảng cách không tối ưu, bỏ qua

            for (auto edge : adjList[u]) {
                double weight = edge.first;
                int v = edge.second;

                if (dist[u] + weight < dist[v]) { // Nếu tìm được khoảng cách ngắn hơn
                    dist[v] = dist[u] + weight;
                    pre[v] = u; // Lưu đỉnh trước đó
                    pq.push({dist[v], v});
                }
            }
        }
    }

    // Truy vết đường đi ngắn nhất từ source đến dest
    vector<int> getPath(int dest) {
        vector<int> path;
        for (int v = dest; v != -1; v = pre[v])
            path.push_back(v);
        reverse(path.begin(), path.end());
        return path;
    }

    // Hàm in kết quả
    void printResult(int source, int dest) {
        if (dist[dest] == INF) {
            cout << "Khong co duong di tu " << char('A' + source) << " den " << char('A' + dest) << endl;
        } else {
            cout << "Khoang cach ngan nhat tu " << char('A' + source) << " den " << char('A' + dest) << ": " << dist[dest] << endl;

            // In đường đi
            vector<int> path = getPath(dest);
            cout << "Duong di: ";
            for (int i = 0; i < path.size(); i++) {
                if (i > 0) cout << " -> ";
                cout << char('A' + path[i]);
            }
            cout << endl;
        }
    }
    
};

int main() {
    // Tọa độ các điểm A, B, C, D, E
    vector<pair<int, int>> points = {
        {1, 2}, {2, 3}, {4, 5}, {19, 45}, {21, 45}
    };

    // Khởi tạo đồ thị với 5 đỉnh
    Graph g(5, points);

    // Thêm các liên kết (cạnh) vào đồ thị (2 chiều)
    g.addEdge(0, 4); // A -> E
    g.addEdge(1, 2); // B -> C
    g.addEdge(1, 4); // B -> E
    g.addEdge(3, 4); // D -> E

    // Nhập điểm nguồn và đích
    char sourceChar, destChar;
    cout << "Nhap diem nguon (A-E): ";
    cin >> sourceChar;
    cout << "Nhap diem dich (A-E): ";
    cin >> destChar;

    int source = sourceChar - 'A'; // Chuyển từ ký tự sang chỉ số
    int dest = destChar - 'A';

    // Chạy thuật toán Dijkstra và in kết quả
    g.dijkstra(source);
    g.printResult(source, dest);

    return 0;
}
