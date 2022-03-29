#include <string>
#include <vector>
#include <iostream>
#include <queue>
#include <functional>
#include <algorithm>

struct Edge {
    Edge(int to, int cost) : to(to), cost(cost) {}

    int to;
    int cost;

    bool operator<(const Edge& other) const {
        return to == other.to ? cost < other.cost : to < other.to;
    }
};

struct MetroBranch {
    MetroBranch(const std::vector<int>& vertex = {}, int interval = 1)
        : vertex(vertex), interval(interval) {}

    size_t size() const {
        return vertex.size();
    }

    std::vector<int> vertex;
    int interval;
};

struct BranchWay {
    BranchWay(const std::vector<int>& vertexInter, const std::vector<int>& branches, int64_t allTime)
        : vertexInter(vertexInter), branches(branches), allTime(allTime) {}

    std::vector<int> vertexInter;
    std::vector<int> branches;
    int64_t allTime;
};

class Metro {
public:
    Metro(std::vector<std::vector<Edge>> graph = {}, std::vector<MetroBranch> branches = {})
        : metroGraph_(graph), branches_(branches), metroSize(graph.size()) {

        for (auto& edges : metroGraph_) {
            std::sort(edges.begin(), edges.end());
        }

        size_t sumSize = metroSize;
        for (auto branch : branches_) {
            sumSize += 2 * branch.size();
        }

        graph_.reserve(sumSize);
        branchByVertex.reserve(sumSize);
        idxInBranch.reserve(sumSize);
        timeIncome.reserve(sumSize);
        minTimeStart.reserve(sumSize);

        graph_.resize(metroSize);
        branchByVertex.resize(metroSize, -1);
        idxInBranch.resize(metroSize, -1);
        timeIncome.resize(metroSize, 0);
        minTimeStart.resize(metroSize, 0);

        for (size_t idx = 0; idx < branches_.size(); idx++) {
            MetroBranch& branch = branches_[idx];
            for (int dir = 0; dir < 2; dir++) {
                int64_t summaryCost = 0;
                for (size_t idxBr = 0; idxBr < branch.size(); idxBr++) {
                    int newVertex = graph_.size();

                    graph_.push_back({ Edge(branch.vertex[idxBr], 0) });
                    graph_[branch.vertex[idxBr]].push_back(Edge(newVertex, 0));

                    branchByVertex.push_back(idx);
                    idxInBranch.push_back(dir == 0 ? idxBr : branch.size() - 1 - idxBr);
                    timeIncome.push_back(summaryCost % branch.interval);
                    minTimeStart.push_back(summaryCost);

                    if (idxBr + 1 != branch.size()) {
                        int edgeCost = getMetroTime(branch.vertex[idxBr], branch.vertex[idxBr + 1]);
                        graph_[newVertex].push_back(Edge(newVertex + 1, edgeCost));
                        summaryCost += edgeCost;
                    }
                }

                std::reverse(branch.vertex.begin(), branch.vertex.end());
            }
        }
    }

    BranchWay getMinimalWay(int st, int fn, int startTime) const {
        std::vector<long long> dist(graph_.size(), -1);
        std::vector<int> parent(graph_.size(), -1);

        std::priority_queue<std::pair<int64_t, int>, std::vector<std::pair<int64_t, int>>, std::greater<std::pair<int64_t, int>>> queue;
        dist[st] = startTime;
        queue.push({ startTime, st });
        while (queue.size() > 0) {
            auto [d, v] = queue.top();
            queue.pop();
            if (dist[v] != d)
                continue;
            for (auto [to, cost] : graph_[v]) {
                if (v >= metroSize && to >= metroSize) {
                    d = getFixedRemaider(d, timeIncome[v], branches_[branchByVertex[v]].interval);
                    d = std::max(d, minTimeStart[v]);
                }
                if (dist[to] == -1 || dist[to] > d + cost) {
                    dist[to] = d + cost;
                    parent[to] = v;
                    queue.push({ dist[to], to });
                }
            }
        }

        if (dist[fn] == -1)
            return BranchWay({}, {}, -1);

        std::vector<int> way;
        int currentVertex = fn;
        while (currentVertex != -1) {
            way.push_back(currentVertex);
            currentVertex = parent[currentVertex];
        }

        std::reverse(way.begin(), way.begin());
        BranchWay ans({}, {}, dist[fn]);
        bool needPushBranch = true;
        for (size_t i = 0; i < way.size(); i++) {
            if (way[i] < metroSize) {
                ans.vertexInter.push_back(way[i]);
                needPushBranch = true;
            } else {
                if (needPushBranch) {
                    ans.branches.push_back(branchByVertex[way[i]]);
                    needPushBranch = false;
                }
            }
        }
        ans.vertexInter.push_back(fn);
        return ans;
    }

private:
    inline static int64_t getFixedRemaider(int64_t value, int r, int mod) {
        if (value % mod <= r)
            return value - value % mod + r;
        else
            return value - value % mod + mod + r;
    }

    int getMetroTime(int v, int u) const {
        if (metroGraph_[v].size() > metroGraph_[u].size())
            std::swap(u, v);

        size_t pos = lower_bound(metroGraph_[v].begin(), metroGraph_[v].end(), Edge(u, 0)) - metroGraph_[v].begin();
        if (pos >= metroGraph_[v].size() || metroGraph_[v][pos].to != u)
            return -1;
        return metroGraph_[v][pos].cost;
    }

    std::vector<std::vector<Edge>> metroGraph_;
    std::vector<MetroBranch> branches_;

    std::vector<std::vector<Edge>> graph_;
    std::vector<int> branchByVertex;
    std::vector<int> idxInBranch;
    std::vector<int> timeIncome;
    std::vector<int64_t> minTimeStart;

    size_t metroSize;
};

int main() {
    int n, m;
    std::cin >> n >> m;
    std::vector<std::vector<Edge>> G(n);
    for (int i = 0; i < m; i++) {
        int u, v, c;
        std::cin >> u >> v >> c;
        u--, v--;
        G[u].emplace_back(v, c);
        G[v].emplace_back(u, c);
    }
    int brCnt;
    std::cin >> brCnt;
    std::vector<MetroBranch> branches(brCnt);
    for (int i = 0; i < brCnt; i++) {
        int sz, interval;
        std::cin >> sz >> interval;
        branches[i].vertex.resize(sz);
        for (auto& u : branches[i].vertex) {
            std::cin >> u;
            u--;
        }
        branches[i].interval = interval;
    }
    Metro metro(G, branches);
    int q;
    std::cin >> q;
    for (int i = 0; i < q; i++) {
        int u, v, t;
        std::cin >> u >> v >> t;
        u--, v--;
        auto ans = metro.getMinimalWay(u, v, t);
        if (ans.allTime != -1) {
            std::cout << "time arriving: " << ans.allTime << std::endl;
            std::cout << "start at " << ans.vertexInter[0] + 1 << std::endl;
            for (size_t i = 0; i + 1 < ans.branches.size(); i++) {
                std::cout << "At station " << ans.vertexInter[i + 1] + 1 << " change branch for " << ans.branches[i] + 1 << std::endl;
            }
            std::cout << "Use branch " << ans.branches.back() + 1 << " for getting " << ans.vertexInter.back() + 1 << " station" << std::endl;
            std::cout << std::endl;
        } else {
            std::cout << "Unreachable\n" << std::endl;
        }
    }
}