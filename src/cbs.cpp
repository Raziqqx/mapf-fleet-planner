// Conflict-Based Search and its bounded-suboptimal variant ECBS.
//
// One engine serves both. The high level keeps the constraint tree in three
// ordered sets: OPEN by lower bound, the same nodes by sum of costs, and FOCAL,
// the nodes whose cost is within w of the smallest lower bound, ordered by
// fewest conflicts. With w == 1 the low level is optimal (cost == lower bound),
// so FOCAL holds exactly the cheapest nodes and the search is plain CBS that
// breaks cost ties by fewest conflicts.
#include <climits>
#include <cmath>
#include <deque>
#include <set>
#include <stdexcept>
#include <tuple>

#include "conflicts.hpp"
#include "space_time.hpp"

namespace mapf {
namespace {

// Slack for comparing integer costs against w * bound in floating point.
constexpr double kEps = 1e-9;

struct Constraint {
    bool edge = false;
    Cell c1 = -1;  // the vertex, or the move's origin
    Cell c2 = -1;  // the move's destination (edge constraints only)
    int t = -1;
};

// A constraint-tree node stores only what changed relative to its parent:
// one agent's new constraint and new path. Full state is rebuilt by walking up.
struct HLNode {
    int parent = -1;
    int agent = -1;  // -1 for the root
    Constraint constraint;
    Path path;
    int path_lb = 0;
    long soc = 0;
    long lb = 0;
    detail::ConflictSummary conflicts;
};

class CbsSearch {
public:
    CbsSearch(const Instance& instance, double w, const Budget& budget)
        : instance_(instance), w_(w), budget_(budget), deadline_(budget) {}

    Result run() {
        if (!detail::precheck(instance_, dist_)) return done(Status::Infeasible);
        if (!plan_root()) return done(result_.status);

        while (!open_lb_.empty()) {
            if (deadline_.expired()) return done(Status::TimeLimit);
            if (result_.stats.hl_expanded >= budget_.max_hl_nodes) return done(Status::NodeLimit);
            const long lb_min = open_lb_.begin()->first;
            raise_focal_bound(lb_min);

            // FOCAL is never empty while OPEN is not (the min-lb node always
            // qualifies); fall back to OPEN only to guard against rounding.
            const int id = focal_.empty() ? open_lb_.begin()->second : std::get<2>(*focal_.begin());
            const HLNode& node = nodes_[static_cast<std::size_t>(id)];
            focal_.erase({node.conflicts.count, node.soc, id});
            open_lb_.erase({node.lb, id});
            open_soc_.erase({node.soc, id});
            ++result_.stats.hl_expanded;

            if (node.conflicts.count == 0) {
                result_.lower_bound = lb_min;
                std::vector<const Path*> paths;
                std::vector<int> lbs;
                collect(id, paths, lbs);
                for (const Path* p : paths) result_.paths.push_back(*p);
                return done(Status::Solved);
            }
            if (!expand(id)) return done(Status::TimeLimit);
        }
        // The whole constraint tree was exhausted: no solution exists.
        return done(Status::Infeasible);
    }

private:
    Result done(Status status) {
        result_.status = status;
        detail::finish(result_, deadline_);
        return std::move(result_);
    }

    // Plans every agent independently (each avoiding, softly, the ones before it).
    bool plan_root() {
        HLNode root;
        detail::ConflictAvoidanceTable cat;
        const detail::ConstraintTable none;
        for (std::size_t i = 0; i < instance_.agents.size(); ++i) {
            if (deadline_.expired()) {
                result_.status = Status::TimeLimit;
                return false;
            }
            const Agent& a = instance_.agents[i];
            auto ll = detail::focal_search(instance_.grid, dist_[i], a.start, a.goal, none, &cat,
                                           w_, deadline_, result_.stats);
            if (!ll.found) {
                // Precheck proved reachability, so only the clock can stop us here.
                result_.status = ll.out_of_time ? Status::TimeLimit : Status::Infeasible;
                return false;
            }
            cat.add_path(ll.path);
            root.soc += path_cost(ll.path);
            root.lb += ll.lower_bound;
            root_lbs_.push_back(ll.lower_bound);
            root_paths_.push_back(std::move(ll.path));
        }
        std::vector<const Path*> paths;
        for (const Path& p : root_paths_) paths.push_back(&p);
        root.conflicts = detail::find_conflicts(paths);
        push(std::move(root));
        return true;
    }

    // Branches on the earliest conflict of node `id`. Returns false on timeout.
    bool expand(int id) {
        const detail::Conflict conflict = nodes_[static_cast<std::size_t>(id)].conflicts.earliest;
        for (int side = 0; side < 2; ++side) {
            const int agent = side == 0 ? conflict.a1 : conflict.a2;
            Constraint c{conflict.edge, conflict.c1, conflict.c2, conflict.t};
            if (conflict.edge && side == 1) std::swap(c.c1, c.c2);

            std::vector<const Path*> paths;
            std::vector<int> lbs;
            collect(id, paths, lbs);
            detail::ConstraintTable table = constraints_for(id, agent);
            add_to_table(table, c);
            detail::ConflictAvoidanceTable cat;
            for (std::size_t j = 0; j < paths.size(); ++j)
                if (static_cast<int>(j) != agent) cat.add_path(*paths[j]);

            const Agent& a = instance_.agents[static_cast<std::size_t>(agent)];
            auto ll =
                detail::focal_search(instance_.grid, dist_[static_cast<std::size_t>(agent)],
                                     a.start, a.goal, table, &cat, w_, deadline_, result_.stats);
            if (ll.out_of_time) return false;
            if (!ll.found) continue;  // this branch has no solution

            const HLNode& parent = nodes_[static_cast<std::size_t>(id)];
            HLNode child;
            child.parent = id;
            child.agent = agent;
            child.constraint = c;
            child.path_lb = ll.lower_bound;
            child.soc = parent.soc - path_cost(*paths[static_cast<std::size_t>(agent)]) +
                        path_cost(ll.path);
            child.lb = parent.lb - lbs[static_cast<std::size_t>(agent)] + ll.lower_bound;
            child.path = std::move(ll.path);
            paths[static_cast<std::size_t>(agent)] = &child.path;
            child.conflicts = detail::find_conflicts(paths);
            push(std::move(child));
        }
        return true;
    }

    // The current path (and its lower bound) of every agent at node `id`:
    // the nearest ancestor that replanned the agent, else the root.
    void collect(int id, std::vector<const Path*>& paths, std::vector<int>& lbs) const {
        const std::size_t n = instance_.agents.size();
        paths.assign(n, nullptr);
        lbs.assign(n, 0);
        for (int i = id; i > 0; i = nodes_[static_cast<std::size_t>(i)].parent) {
            const HLNode& node = nodes_[static_cast<std::size_t>(i)];
            const auto a = static_cast<std::size_t>(node.agent);
            if (paths[a] == nullptr) {
                paths[a] = &node.path;
                lbs[a] = node.path_lb;
            }
        }
        for (std::size_t a = 0; a < n; ++a) {
            if (paths[a] == nullptr) {
                paths[a] = &root_paths_[a];
                lbs[a] = root_lbs_[a];
            }
        }
    }

    detail::ConstraintTable constraints_for(int id, int agent) const {
        detail::ConstraintTable table;
        for (int i = id; i > 0; i = nodes_[static_cast<std::size_t>(i)].parent) {
            const HLNode& node = nodes_[static_cast<std::size_t>(i)];
            if (node.agent == agent) add_to_table(table, node.constraint);
        }
        return table;
    }

    static void add_to_table(detail::ConstraintTable& table, const Constraint& c) {
        if (c.edge)
            table.add_edge(c.c1, c.c2, c.t);
        else
            table.add_vertex(c.c1, c.t);
    }

    void push(HLNode node) {
        const int id = static_cast<int>(nodes_.size());
        open_lb_.emplace(node.lb, id);
        open_soc_.emplace(node.soc, id);
        if (static_cast<double>(node.soc) <= focal_bound_ + kEps)
            focal_.emplace(node.conflicts.count, node.soc, id);
        nodes_.push_back(std::move(node));  // deque: earlier nodes' paths stay put
        ++result_.stats.hl_generated;
    }

    // FOCAL admits every open node with soc <= w * lb_min. lb_min never
    // decreases, so the bound only grows and nodes are only ever added.
    void raise_focal_bound(long lb_min) {
        const double bound = w_ * static_cast<double>(lb_min);
        if (bound <= focal_bound_) return;
        for (auto it = open_soc_.upper_bound(
                 {static_cast<long>(std::floor(focal_bound_ + kEps)), INT_MAX});
             it != open_soc_.end() && static_cast<double>(it->first) <= bound + kEps; ++it) {
            const HLNode& n = nodes_[static_cast<std::size_t>(it->second)];
            focal_.emplace(n.conflicts.count, n.soc, it->second);
        }
        focal_bound_ = bound;
    }

    const Instance& instance_;
    const double w_;
    const Budget budget_;
    detail::Deadline deadline_;
    Result result_;
    std::vector<std::vector<int>> dist_;
    std::vector<Path> root_paths_;
    std::vector<int> root_lbs_;
    std::deque<HLNode> nodes_;
    std::set<std::pair<long, int>> open_lb_;
    std::set<std::pair<long, int>> open_soc_;
    std::set<std::tuple<int, long, int>> focal_;  // (conflicts, soc, id)
    double focal_bound_ = -1.0;
};

}  // namespace

Result plan_cbs(const Instance& instance, const Budget& budget) {
    return CbsSearch(instance, 1.0, budget).run();
}

Result plan_ecbs(const Instance& instance, double w, const Budget& budget) {
    if (!(w >= 1.0)) throw std::invalid_argument("ECBS suboptimality w must be >= 1");
    return CbsSearch(instance, w, budget).run();
}

}  // namespace mapf
