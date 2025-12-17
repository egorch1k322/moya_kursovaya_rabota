#include <iostream>   // Ввод/вывод
#include <vector>     // Контейнер vector
#include <queue>      // Очередь с приоритетом
#include <cmath>      // Математические функции (sqrt, fabs)
#include <algorithm> // Алгоритмы STL
#include <stack>     // Стек
#include <random>    // Генерация случайных чисел
#include <chrono>    // Измерение времени
using namespace std;

// Структура Node используется в очереди с приоритетом для реализации алгоритма A*.
struct Node {
    int v;        // Номер вершины
    double f;     // Значение f = g + h (оценка стоимости пути)

    // Переопределение оператора < для priority_queue
    // priority_queue в C++ является max-heap, поэтому
    // используется обратное сравнение
    bool operator<(const Node& other) const {
        return f > other.f;
    }
};

// Структура Edge описывает ребро графа
struct Edge {
    int to;         // Вершина назначения
    double weight;  // Вес ребра
};

// Класс Graph реализует граф и алгоритмы поиска пути
class Graph {
private:
    int n;                                      // Количество вершин
    vector<vector<Edge>> adj;                  // Список смежности
    vector<pair<double, double>> pos;           // Координаты вершин (x, y)
    bool directed;                              // Флаг ориентированности графа

    // Восстановление пути от стартовой вершины к целевой на основе массива cameFrom
    vector<int> reconstructPath(const vector<int>& cameFrom, int cur) {
        vector<int> path = { cur };
        while (cameFrom[cur] != -1) {
            cur = cameFrom[cur];
            path.insert(path.begin(), cur);
        }
        return path;
    }

     // Эвристики для алгоритма A*
     // Евклидово расстояние между двумя вершинами
    double euclidean(int a, int b) const {
        double dx = pos[a].first - pos[b].first;
        double dy = pos[a].second - pos[b].second;
        return sqrt(dx * dx + dy * dy);
    }

    // Манхэттенское расстояние
    double manhatten(int a, int b) const {
        double dx = pos[a].first - pos[b].first;
        double dy = pos[a].second - pos[b].second;
        return fabs(dx) + fabs(dy);
    }

    // Расстояние Чебышева
    double Chebyshev_distance(int a, int b) const {
        double dx = pos[a].first - pos[b].first;
        double dy = pos[a].second - pos[b].second;
        return max(fabs(dx), fabs(dy));
    }

    // Нулевая эвристика (эквивалентна алгоритму Дейкстры)
    double heuristic_zero(int a, int b) {
        return 0 * (a + b);
    }

    // Завышенная эвристика (неадмиссимая)
    double heuristic_over(int a, int b) {
        return 5 * manhatten(a, b);
    }

public:
    // Конструктор графа
    // Генерирует случайные координаты вершин в диапазоне [0;100]
    Graph(int n, mt19937& rng, bool f) : n(n), adj(n), pos(n), directed(f) {
        uniform_real_distribution<double> coordDist(0.0, 100.0);
        for (int i = 0; i < n; ++i) {
            pos[i] = { coordDist(rng), coordDist(rng) };
        }
    }

    // Добавление ребра в граф
    // Вес ребра равен евклидову расстоянию между вершинами
    void addEdge(int u, int v) {
        double w = euclidean(u, v);
        adj[u].push_back({ v, w });
        if (!directed)
            adj[v].push_back({ u, w });
    }

    // Поиск пути с помощью алгоритма DFS (поиск в глубину)
    vector<int> DFS(int start, int goal) {
        vector<int> cameFrom(n, -1);      // Массив родителей
        vector<bool> visited(n, false);   // Массив посещённых вершин
        stack<pair<int, int>> st;          // Стек для DFS

        st.push({ start, -1 });
        visited[start] = true;

        while (!st.empty()) {
            auto topElem = st.top();
            st.pop();

            int current = topElem.first;

            // Если достигли цели — восстанавливаем путь
            if (current == goal) {
                return reconstructPath(cameFrom, goal);
            }

            // Обход всех соседей текущей вершины
            for (const auto& edge : adj[current]) {
                int v = edge.to;
                if (!visited[v]) {
                    visited[v] = true;
                    cameFrom[v] = current;
                    st.push({ v, current });
                }
            }
        }
        return {}; // Путь не найден
    }

    // Алгоритм A* (A-Star)
    vector<int> A_Star(int start, int goal) {
        vector<int> cameFrom(n, -1);       // Массив родителей
        vector<double> gScore(n, 1e18);    // Стоимость пути от старта
        vector<double> fScore(n, 1e18);    // g + h
        vector<bool> visited(n, false);    // Посещённые вершины

        gScore[start] = 0;
        fScore[start] = euclidean(start, goal);

        priority_queue<Node> openSet;
        openSet.push({ start, fScore[start] });

        while (!openSet.empty()) {
            int current = openSet.top().v;
            openSet.pop();

            if (visited[current]) continue;
            visited[current] = true;

            if (current == goal)
                return reconstructPath(cameFrom, current);

            for (const auto& edge : adj[current]) {
                int v = edge.to;
                double tentative = gScore[current] + edge.weight;

                if (tentative < gScore[v]) {
                    cameFrom[v] = current;
                    gScore[v] = tentative;
                    fScore[v] = tentative + euclidean(v, goal);
                    openSet.push({ v, fScore[v] });
                }
            }
        }
        return {}; // Путь не найден
    }

    // Подсчёт длины пути
    double pathLength(const vector<int>& path) const {
        if (path.size() < 2) return 0.0;

        double sum = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            for (const auto& edge : adj[path[i]]) {
                if (edge.to == path[i + 1]) {
                    sum += edge.weight;
                    break;
                }
            }
        }
        return sum;
    }
};

// Точка входа в программу

int main() {
    setlocale(LC_ALL, "rus");

    const int TESTS = 100;  // Количество тестов
    const int N = 100;        // Количество вершин
    const int EDGES = 10000;    // Количество дополнительных рёбер

    mt19937 rng(12345);
    uniform_int_distribution<int> vertexDist(0, N - 1);

    double totalDFS = 0;
    double totalAStar = 0;
    bool comp = true;

    for (int test = 0; test < TESTS; test++) {
        Graph g(N, rng, false);

        // Создание связного графа
        for (int i = 0; i < N - 1; i++) {
            g.addEdge(i, i + 1);
        }

        // Добавление случайных рёбер
        for (int i = 0; i < EDGES; i++) {
            int u = vertexDist(rng);
            int v = vertexDist(rng);
            if (u != v) {
                g.addEdge(u, v);
            }
        }

        int start = vertexDist(rng);
        int goal = vertexDist(rng);
        while (goal == start)
            goal = vertexDist(rng);

        // Измерение времени DFS
        auto t1 = chrono::high_resolution_clock::now();
        auto dfs_Path = g.DFS(start, goal);
        auto t2 = chrono::high_resolution_clock::now();
        totalDFS += chrono::duration<double, milli>(t2 - t1).count();

        // Измерение времени A*
        t1 = chrono::high_resolution_clock::now();
        auto A_star_Path = g.A_Star(start, goal);
        t2 = chrono::high_resolution_clock::now();
        totalAStar += chrono::duration<double, milli>(t2 - t1).count();

        if (dfs_Path.empty() || A_star_Path.empty()) {
            cout << "Ошибка: путь не найден\n";
            return 1;
        }

        if (g.pathLength(A_star_Path) > g.pathLength(dfs_Path))
            comp = false;
    }

    // Вывод результатов
    cout << "Количество вершин: " << N << endl;
    cout << "Количество рёбер: " << EDGES << endl;
    cout << "Количество тестов: " << TESTS << endl;
    cout << "Среднее время DFS: " << totalDFS / TESTS << " мс\n";
    cout << "Среднее время A*: " << totalAStar / TESTS << " мс\n";

    if (comp)
        cout << "A* всегда находит путь не хуже DFS\n";
    else
        cout << "A* в некоторых случаях хуже DFS\n";

    return 0;
}

