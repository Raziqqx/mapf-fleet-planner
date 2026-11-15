// Single-agent space-time focal search (see focal_search in space_time.hpp).
//
// The search is written once (search_impl) and instantiated for each node
// layout and open-list implementation selected by mapf::SearchTuning, so the
// profiling script can compare them on one build. All variants order nodes by
// the same total keys and therefore expand identical states in identical order.
#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <memory>
#include <memory_resource>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>

#include "space_time.hpp"

namespace mapf {
namespace {
SearchTuning g_tuning;
}  // namespace

void set_search_tuning(const SearchTuning& tuning) { g_tuning = tuning; }
SearchTuning search_tuning() { return g_tuning; }

const char* to_string(OpenList open_list) {
    switch (open_list) {
        case OpenList::Set: return "set";
        case OpenList::BinaryHeap: return "heap";
        case OpenList::Bucket: return "bucket";
    }
    return "?";
}
const char* to_string(NodeLayout layout) { return layout == NodeLayout::AoS ? "aos" : "soa"; }

}  // namespace mapf

namespace mapf::detail {
namespace {

// Poll the clock once per this many expansions.
constexpr std::uint64_t kClockPollMask = 255;

// ---------------------------------------------------------------------------
// Node storage. Nodes are addressed by dense int ids; f and t never change
// after creation, conflicts/parent improve while open, closed is set once.

class AosNodes {
public:
    explicit AosNodes(std::pmr::memory_resource* mr) : nodes_(mr) {}
    int add(Cell cell, int t, int f, int conflicts, int parent) {
        nodes_.push_back(Node{cell, t, f, conflicts, parent, false});
        return static_cast<int>(nodes_.size()) - 1;
    }
    Cell cell(int i) const { return at(i).cell; }
    int t(int i) const { return at(i).t; }
    int f(int i) const { return at(i).f; }
    int conflicts(int i) const { return at(i).conflicts; }
    int parent(int i) const { return at(i).parent; }
    bool closed(int i) const { return at(i).closed; }
    void improve(int i, int conflicts, int parent) {
        Node& n = nodes_[static_cast<std::size_t>(i)];
        n.conflicts = conflicts;
        n.parent = parent;
    }
    void close(int i) { nodes_[static_cast<std::size_t>(i)].closed = true; }

private:
    struct Node {
        Cell cell;
        int t;  // also g: every action, waiting included, costs 1
        int f;
        int conflicts;  // soft conflicts accumulated along the best known path here
        int parent;     // node id, -1 for the root
        bool closed;
    };
    const Node& at(int i) const { return nodes_[static_cast<std::size_t>(i)]; }
    std::pmr::vector<Node> nodes_;
};

class SoaNodes {
public:
    explicit SoaNodes(std::pmr::memory_resource* mr)
        : cell_(mr), t_(mr), f_(mr), conflicts_(mr), parent_(mr), closed_(mr) {}
    int add(Cell cell, int t, int f, int conflicts, int parent) {
        cell_.push_back(cell);
        t_.push_back(t);
        f_.push_back(f);
        conflicts_.push_back(conflicts);
        parent_.push_back(parent);
        closed_.push_back(0);
        return static_cast<int>(cell_.size()) - 1;
    }
    Cell cell(int i) const { return cell_[idx(i)]; }
    int t(int i) const { return t_[idx(i)]; }
    int f(int i) const { return f_[idx(i)]; }
    int conflicts(int i) const { return conflicts_[idx(i)]; }
    int parent(int i) const { return parent_[idx(i)]; }
    bool closed(int i) const { return closed_[idx(i)] != 0; }
    void improve(int i, int conflicts, int parent) {
        conflicts_[idx(i)] = conflicts;
        parent_[idx(i)] = parent;
    }
    void close(int i) { closed_[idx(i)] = 1; }

private:
    static std::size_t idx(int i) { return static_cast<std::size_t>(i); }
    std::pmr::vector<Cell> cell_;
    std::pmr::vector<int> t_, f_, conflicts_, parent_;
    std::pmr::vector<std::uint8_t> closed_;
};

// ---------------------------------------------------------------------------
// Open/focal lists. OPEN holds every unexpanded node (for f_min); FOCAL holds
// the open nodes with f <= bound = w * f_min, ordered by (conflicts, f, -t, id).
// Interface used by search_impl:
//   push(id)            a new open node (goes to FOCAL iff f <= bound)
//   improve(id, old_c)  an open node's conflicts dropped from old_c
//   f_min()             smallest f among open nodes (at least one exists)
//   raise_bound(b)      pull the nodes with bound < f <= b into FOCAL
//   pop()               remove and return the best FOCAL node (caller closes it)

using FocalKey = std::tuple<int, int, int, int>;  // (conflicts, f, -t, id)

template <class Nodes>
FocalKey focal_key(const Nodes& nodes, int id) {
    return {nodes.conflicts(id), nodes.f(id), -nodes.t(id), id};
}

// Original implementation: balanced trees, exact deletion.
template <class Nodes>
class SetQueue {
public:
    SetQueue(const Nodes& nodes, double bound, std::pmr::memory_resource* mr)
        : nodes_(nodes), bound_(bound), open_(mr), focal_(mr) {}
    void push(int id) {
        open_.emplace(nodes_.f(id), id);
        if (nodes_.f(id) <= bound_) focal_.insert(focal_key(nodes_, id));
    }
    void improve(int id, int old_conflicts) {
        if (focal_.erase({old_conflicts, nodes_.f(id), -nodes_.t(id), id}) != 0)
            focal_.insert(focal_key(nodes_, id));
    }
    int f_min() const { return open_.begin()->first; }
    double bound() const { return bound_; }
    void raise_bound(double new_bound) {
        for (auto it = open_.upper_bound({static_cast<int>(bound_), INT_MAX});
             it != open_.end() && it->first <= new_bound; ++it)
            focal_.insert(focal_key(nodes_, it->second));
        bound_ = new_bound;
    }
    int pop() {
        const int id = std::get<3>(*focal_.begin());
        focal_.erase(focal_.begin());
        open_.erase({nodes_.f(id), id});
        return id;
    }

private:
    using OpenKey = std::pair<int, int>;  // (f, id)
    const Nodes& nodes_;
    double bound_;
    std::pmr::set<OpenKey> open_;
    std::pmr::set<FocalKey> focal_;
};

template <class T>
using MinHeap = std::priority_queue<T, std::pmr::vector<T>, std::greater<>>;

// FOCAL as a binary heap with lazy deletion: an improved node is pushed again
// and the stale entry (higher conflict count) is skipped when it surfaces.
template <class Nodes>
class FocalHeap {
public:
    FocalHeap(const Nodes& nodes, std::pmr::memory_resource* mr)
        : nodes_(nodes), heap_(std::greater<>{}, std::pmr::vector<FocalKey>(mr)) {}
    void push(int id) { heap_.push(focal_key(nodes_, id)); }
    int pop() {
        for (;;) {
            const auto [conflicts, f, neg_t, id] = heap_.top();
            heap_.pop();
            if (!nodes_.closed(id) && nodes_.conflicts(id) == conflicts) return id;
        }
    }

private:
    const Nodes& nodes_;
    MinHeap<FocalKey> heap_;
};

// OPEN as a binary heap of (f, id), lazily dropping closed nodes, plus a
// second heap of the open nodes not yet in FOCAL (f > bound).
template <class Nodes>
class HeapQueue {
public:
    HeapQueue(const Nodes& nodes, double bound, std::pmr::memory_resource* mr)
        : nodes_(nodes),
          bound_(bound),
          open_(std::greater<>{}, std::pmr::vector<OpenKey>(mr)),
          pending_(std::greater<>{}, std::pmr::vector<OpenKey>(mr)),
          focal_(nodes, mr) {}
    void push(int id) {
        const int f = nodes_.f(id);
        open_.emplace(f, id);
        if (f <= bound_)
            focal_.push(id);
        else
            pending_.emplace(f, id);
    }
    // Nodes outside FOCAL are keyed by f only, so nothing to update for them.
    void improve(int id, int /*old_conflicts*/) {
        if (nodes_.f(id) <= bound_) focal_.push(id);
    }
    int f_min() {
        while (nodes_.closed(open_.top().second)) open_.pop();
        return open_.top().first;
    }
    double bound() const { return bound_; }
    void raise_bound(double new_bound) {
        while (!pending_.empty() && pending_.top().first <= new_bound) {
            focal_.push(pending_.top().second);
            pending_.pop();
        }
        bound_ = new_bound;
    }
    int pop() { return focal_.pop(); }

private:
    using OpenKey = std::pair<int, int>;  // (f, id)
    const Nodes& nodes_;
    double bound_;
    MinHeap<OpenKey> open_;
    MinHeap<OpenKey> pending_;
    FocalHeap<Nodes> focal_;
};

// OPEN as a bucket queue over the integer f: a count of open nodes per f
// value and a cursor at the smallest non-empty bucket (f never decreases along
// a path because the heuristic is consistent). Nodes with f > bound wait in
// their bucket until the bound passes it.
template <class Nodes>
class BucketQueue {
public:
    BucketQueue(const Nodes& nodes, double bound, std::pmr::memory_resource* mr)
        : nodes_(nodes), bound_(bound), count_(mr), pending_(mr), focal_(nodes, mr) {}
    void push(int id) {
        const auto f = static_cast<std::size_t>(nodes_.f(id));
        if (f >= count_.size()) {
            count_.resize(f + 1, 0);
            pending_.resize(f + 1);  // inner vectors pick up the allocator (uses-allocator)
        }
        ++count_[f];
        cursor_ = std::min(cursor_, f);
        if (static_cast<double>(f) <= bound_)
            focal_.push(id);
        else
            pending_[f].push_back(id);
    }
    void improve(int id, int /*old_conflicts*/) {
        if (nodes_.f(id) <= bound_) focal_.push(id);
    }
    int f_min() {
        while (count_[cursor_] == 0) ++cursor_;
        return static_cast<int>(cursor_);
    }
    double bound() const { return bound_; }
    void raise_bound(double new_bound) {
        // Buckets above the old bound were never moved to FOCAL; each is moved once.
        const auto hi =
            std::min(static_cast<std::size_t>(std::floor(new_bound)) + 1, pending_.size());
        for (auto f = static_cast<std::size_t>(std::floor(bound_)) + 1; f < hi; ++f) {
            for (int id : pending_[f]) focal_.push(id);
            pending_[f].clear();
        }
        bound_ = new_bound;
    }
    int pop() {
        const int id = focal_.pop();
        --count_[static_cast<std::size_t>(nodes_.f(id))];
        return id;
    }

private:
    const Nodes& nodes_;
    double bound_;
    std::size_t cursor_ = SIZE_MAX;
    std::pmr::vector<int> count_;
    std::pmr::vector<std::pmr::vector<int>> pending_;
    FocalHeap<Nodes> focal_;
};

// ---------------------------------------------------------------------------
// Per-thread arena: a buffer reused by every search on this thread. A search
// that outgrows it spills to the heap (counted) and the buffer is enlarged
// for the next search, so steady state does no heap allocation at all.

class CountingResource : public std::pmr::memory_resource {
public:
    std::size_t allocated = 0;

private:
    void* do_allocate(std::size_t bytes, std::size_t align) override {
        allocated += bytes;
        return std::pmr::new_delete_resource()->allocate(bytes, align);
    }
    void do_deallocate(void* p, std::size_t bytes, std::size_t align) override {
        std::pmr::new_delete_resource()->deallocate(p, bytes, align);
    }
    bool do_is_equal(const memory_resource& other) const noexcept override {
        return this == &other;
    }
};

constexpr std::size_t kArenaInitial = std::size_t{1} << 20;  // 1 MiB
constexpr std::size_t kArenaMax = std::size_t{256} << 20;    // never keep more than 256 MiB

struct Arena {
    std::unique_ptr<std::byte[]> buffer;
    std::size_t size = 0;
};

template <class Nodes, template <class> class Queue>
LowLevelResult search_impl(const Grid& grid, const std::vector<int>& h, Cell start, Cell goal,
                           const ConstraintTable& constraints, const ConflictAvoidanceTable* cat,
                           double w, const Deadline& deadline, Stats& stats, int min_goal_t,
                           std::pmr::memory_resource* mr) {
    LowLevelResult result;
    // Beyond the last constraint the space-time graph repeats, so any goal
    // that is reachable at all is reached within free_count further steps.
    const int horizon =
        std::max(constraints.last_constraint_time(), min_goal_t) + grid.free_count() + 1;
    // Admissible and consistent: the agent cannot finish before min_goal_t.
    const auto heuristic = [&](Cell c, int t) {
        return std::max(h[static_cast<std::size_t>(c)], min_goal_t - t);
    };

    Nodes nodes(mr);
    std::pmr::unordered_map<std::uint64_t, int> index(mr);  // vertex_key(cell, t) -> node
    const int root = nodes.add(start, 0, heuristic(start, 0), 0, -1);
    index.emplace(vertex_key(start, 0), root);
    Queue<Nodes> queue(nodes, w * nodes.f(root), mr);
    queue.push(root);
    std::size_t open_count = 1;

    std::array<Cell, 4> nbrs{};
    std::uint64_t expanded = 0;
    while (open_count > 0) {
        if ((++expanded & kClockPollMask) == 0 && deadline.expired()) {
            result.out_of_time = true;
            break;
        }
        // Raise the focal bound if f_min grew, pulling the newly eligible nodes in.
        const int f_min = queue.f_min();
        if (w * f_min > queue.bound()) queue.raise_bound(w * f_min);
        const int id = queue.pop();
        nodes.close(id);
        --open_count;
        ++stats.ll_expanded;
        const Cell cur_cell = nodes.cell(id);
        const int cur_t = nodes.t(id);

        if (cur_cell == goal && cur_t >= min_goal_t) {
            result.found = true;
            result.lower_bound = f_min;
            result.path.resize(static_cast<std::size_t>(cur_t) + 1);
            for (int i = id; i >= 0; i = nodes.parent(i))
                result.path[static_cast<std::size_t>(nodes.t(i))] = nodes.cell(i);
            break;
        }
        if (cur_t >= horizon) continue;

        const int t = cur_t + 1;
        const int cur_conflicts = nodes.conflicts(id);
        const int n = grid.neighbours(cur_cell, nbrs);
        // k == n is the wait action.
        for (int k = 0; k <= n; ++k) {
            const Cell next = k < n ? nbrs[static_cast<std::size_t>(k)] : cur_cell;
            if (h[static_cast<std::size_t>(next)] < 0) continue;
            if (constraints.vertex_blocked(next, t) || constraints.edge_blocked(cur_cell, next, t))
                continue;
            const int conflicts = cur_conflicts + (cat ? cat->count(cur_cell, next, t) : 0);
            const auto [it, inserted] = index.try_emplace(vertex_key(next, t), -1);
            if (!inserted) {
                // Same (cell, t) means same g and f; only a path with fewer
                // conflicts is an improvement, and only while still open.
                const int old = it->second;
                const int old_conflicts = nodes.conflicts(old);
                if (nodes.closed(old) || conflicts >= old_conflicts) continue;
                nodes.improve(old, conflicts, id);
                queue.improve(old, old_conflicts);
                continue;
            }
            it->second = nodes.add(next, t, t + heuristic(next, t), conflicts, id);
            queue.push(it->second);
            ++open_count;
        }
    }
    return result;
}

template <class Nodes>
LowLevelResult dispatch_queue(OpenList open_list, auto&&... args) {
    switch (open_list) {
        case OpenList::Set: return search_impl<Nodes, SetQueue>(args...);
        case OpenList::BinaryHeap: return search_impl<Nodes, HeapQueue>(args...);
        case OpenList::Bucket: break;
    }
    return search_impl<Nodes, BucketQueue>(args...);
}

}  // namespace

LowLevelResult focal_search(const Grid& grid, const std::vector<int>& h, Cell start, Cell goal,
                            const ConstraintTable& constraints, const ConflictAvoidanceTable* cat,
                            double w, const Deadline& deadline, Stats& stats) {
    ++stats.ll_calls;
    const int min_goal_t = constraints.min_goal_time(goal);
    if (min_goal_t < 0 || constraints.vertex_blocked(start, 0)) return {};
    if (cat != nullptr && cat->empty()) cat = nullptr;

    const SearchTuning tuning = search_tuning();
    const auto run = [&](std::pmr::memory_resource* mr) {
        if (tuning.layout == NodeLayout::SoA)
            return dispatch_queue<SoaNodes>(tuning.open_list, grid, h, start, goal, constraints,
                                            cat, w, deadline, stats, min_goal_t, mr);
        return dispatch_queue<AosNodes>(tuning.open_list, grid, h, start, goal, constraints, cat, w,
                                        deadline, stats, min_goal_t, mr);
    };
    if (!tuning.arena) return run(std::pmr::new_delete_resource());

    thread_local Arena arena;
    if (arena.size == 0) {
        arena.buffer = std::make_unique_for_overwrite<std::byte[]>(kArenaInitial);
        arena.size = kArenaInitial;
    }
    CountingResource spill;
    LowLevelResult result;
    {
        // Monotonic: deallocation is a no-op, everything is dropped at scope end.
        std::pmr::monotonic_buffer_resource mono(arena.buffer.get(), arena.size, &spill);
        result = run(&mono);
    }
    if (spill.allocated > 0 && arena.size < kArenaMax) {
        arena.size = std::min(kArenaMax, std::max(2 * arena.size, arena.size + spill.allocated));
        arena.buffer = std::make_unique_for_overwrite<std::byte[]>(arena.size);
    }
    return result;
}

}  // namespace mapf::detail
