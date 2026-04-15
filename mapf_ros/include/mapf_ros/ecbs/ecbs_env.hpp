#pragma once

#ifndef ECBS_ENV_H
#define ECBS_ENV_H

#include <algorithm>
#include <array>
#include <cmath>

#include "../utils/utility.hpp"

#include "../utils/neighbor.hpp"
#include "../utils/planresult.hpp"

using mapf::Neighbor;
using mapf::PlanResult;

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State &s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

  friend std::ostream &operator<<(std::ostream &os, const State &s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};

namespace std {
template <> struct hash<State> {
  size_t operator()(const State &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream &operator<<(std::ostream &os, const Action &a) {
  switch (a) {
  case Action::Up:
    os << "Up";
    break;
  case Action::Down:
    os << "Down";
    break;
  case Action::Left:
    os << "Left";
    break;
  case Action::Right:
    os << "Right";
    break;
  case Action::Wait:
    os << "Wait";
    break;
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;
  State agent1_start;
  State agent1_end;
  State agent2_start;
  State agent2_end;

  Conflict()
      : time(0), agent1(0), agent2(0), type(Vertex), agent1_start(0, 0, 0),
        agent1_end(0, 0, 0), agent2_start(0, 0, 0), agent2_end(0, 0, 0) {}

  friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
    switch (c.type) {
    case Vertex:
      return os << c.time << ": Vertex(" << c.agent1_start.x << ","
                << c.agent1_start.y << ")";
    case Edge:
      return os << c.time << ": Edge(" << c.agent1_start.x << ","
                << c.agent1_start.y << "," << c.agent1_end.x << ","
                << c.agent1_end.y << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint &other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint &other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const VertexConstraint &c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <> struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream &operator<<(std::ostream &os, const EdgeConstraint &c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <> struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
} // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints &other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints &other) const {
    for (const auto &vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto &ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
    for (const auto &vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto &ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location &other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location &other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const Location &c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <> struct hash<Location> {
  size_t operator()(const Location &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

///
class Environment {
public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = false,
              double minAgentCenterDistance = 0.0)
      : m_dimx(dimx), m_dimy(dimy), m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)), m_agentIdx(0), m_constraints(nullptr),
        m_lastGoalConstraint(-1), m_highLevelExpanded(0), m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal),
        m_minAgentCenterDistance(minAgentCenterDistance),
        m_minAgentCenterDistanceSquared(minAgentCenterDistance *
                                        minAgentCenterDistance) {}

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
    assert(constraints); // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    const auto &goal = m_goals[m_agentIdx];
    for (const auto &vc : constraints->vertexConstraints) {
      if (vc.x == goal.x && vc.y == goal.y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
    for (const auto &ec : constraints->edgeConstraints) {
      if ((ec.x1 == goal.x && ec.y1 == goal.y) ||
          (ec.x2 == goal.x && ec.y2 == goal.y)) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, ec.time);
      }
    }
  }

  int admissibleHeuristic(const State &s) {
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  // low-level
  int focalStateHeuristic(
      const State &s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, int>> &solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        if (statesWithinClearance(s, state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State &s1a, const State &s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, int>> &solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (transitionsWithinClearance(s1a, s1b, s2a, s2b)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, int>> &solution) {
    int numConflicts = 0;

    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (statesWithinClearance(state1, state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (transitionsWithinClearance(state1a, state1b, state2a,
                                         state2b)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State &s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State &s,
                    std::vector<Neighbor<State, Action, int>> &neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool
  getFirstConflict(const std::vector<PlanResult<State, Action, int>> &solution,
                   Conflict &result) {
    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t <= max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (statesWithinClearance(state1, state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.agent1_start = state1;
            result.agent1_end = state1;
            result.agent2_start = state2;
            result.agent2_end = state2;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (transitionsWithinClearance(state1a, state1b, state2a,
                                         state2b)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.agent1_start = state1a;
            result.agent1_end = state1b;
            result.agent2_start = state2a;
            result.agent2_end = state2b;
            return true;
          }
        }
      }
    }

    return false;
  }

  void
  createConstraintsFromConflict(const Conflict &conflict,
                                std::map<size_t, Constraints> &constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      Constraints c2;
      addVertexConstraintsAround(conflict.time, conflict.agent2_start, c1);
      addVertexConstraintsAround(conflict.time, conflict.agent1_start, c2);
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c2;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      Constraints c2;
      addEdgeConstraintsAround(conflict.time, conflict.agent2_start,
                               conflict.agent2_end, c1);
      addEdgeConstraintsAround(conflict.time, conflict.agent1_start,
                               conflict.agent1_end, c2);
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

private:
  bool statesWithinClearance(const State &state1, const State &state2) const {
    if (m_minAgentCenterDistance <= 0.0) {
      return state1.equalExceptTime(state2);
    }

    const double dx = static_cast<double>(state1.x - state2.x);
    const double dy = static_cast<double>(state1.y - state2.y);
    return dx * dx + dy * dy <= m_minAgentCenterDistanceSquared;
  }

  bool transitionsWithinClearance(const State &state1a, const State &state1b,
                                  const State &state2a,
                                  const State &state2b) const {
    if (m_minAgentCenterDistance <= 0.0) {
      return state1a.equalExceptTime(state2b) &&
             state1b.equalExceptTime(state2a);
    }

    return segmentDistanceSquared(
               static_cast<double>(state1a.x), static_cast<double>(state1a.y),
               static_cast<double>(state1b.x), static_cast<double>(state1b.y),
               static_cast<double>(state2a.x), static_cast<double>(state2a.y),
               static_cast<double>(state2b.x),
               static_cast<double>(state2b.y)) <=
           m_minAgentCenterDistanceSquared;
  }

  void addVertexConstraintsAround(int time, const State &otherState,
                                  Constraints &constraints) const {
    if (m_minAgentCenterDistance <= 0.0) {
      constraints.vertexConstraints.emplace(
          VertexConstraint(time, otherState.x, otherState.y));
      return;
    }

    const int radius = static_cast<int>(std::ceil(m_minAgentCenterDistance));
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        const double distSquared = static_cast<double>(dx * dx + dy * dy);
        if (distSquared <= m_minAgentCenterDistanceSquared) {
          constraints.vertexConstraints.emplace(
              VertexConstraint(time, otherState.x + dx, otherState.y + dy));
        }
      }
    }
  }

  void addEdgeConstraintsAround(int time, const State &otherStart,
                                const State &otherEnd,
                                Constraints &constraints) const {
    if (m_minAgentCenterDistance <= 0.0) {
      constraints.edgeConstraints.emplace(EdgeConstraint(
          time, otherStart.x, otherStart.y, otherEnd.x, otherEnd.y));
      return;
    }

    const int radius =
        static_cast<int>(std::ceil(m_minAgentCenterDistance)) + 1;
    const int minX = std::min(otherStart.x, otherEnd.x) - radius;
    const int maxX = std::max(otherStart.x, otherEnd.x) + radius;
    const int minY = std::min(otherStart.y, otherEnd.y) - radius;
    const int maxY = std::max(otherStart.y, otherEnd.y) + radius;
    const std::array<std::pair<int, int>, 5> motionDeltas = {
        std::make_pair(0, 0), std::make_pair(-1, 0), std::make_pair(1, 0),
        std::make_pair(0, 1), std::make_pair(0, -1)};

    for (int x = minX; x <= maxX; ++x) {
      for (int y = minY; y <= maxY; ++y) {
        for (const auto &delta : motionDeltas) {
          const int nextX = x + delta.first;
          const int nextY = y + delta.second;
          if (segmentDistanceSquared(
                  static_cast<double>(x), static_cast<double>(y),
                  static_cast<double>(nextX), static_cast<double>(nextY),
                  static_cast<double>(otherStart.x),
                  static_cast<double>(otherStart.y),
                  static_cast<double>(otherEnd.x),
                  static_cast<double>(otherEnd.y)) <=
              m_minAgentCenterDistanceSquared) {
            constraints.edgeConstraints.emplace(
                EdgeConstraint(time, x, y, nextX, nextY));
          }
        }
      }
    }
  }

  double segmentDistanceSquared(double ax, double ay, double bx, double by,
                                double cx, double cy, double dx,
                                double dy) const {
    const double rvx = bx - ax;
    const double rvy = by - ay;
    const double svx = dx - cx;
    const double svy = dy - cy;
    const double qx = ax - cx;
    const double qy = ay - cy;
    const double dvx = rvx - svx;
    const double dvy = rvy - svy;
    const double a = dvx * dvx + dvy * dvy;

    double t = 0.0;
    if (a > 1e-9) {
      t = -(qx * dvx + qy * dvy) / a;
      t = std::clamp(t, 0.0, 1.0);
    }

    const double diffX = qx + dvx * t;
    const double diffY = qy + dvy * t;
    return diffX * diffX + diffY * diffY;
  }

  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1 * (agentIdx + 1), -1);
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {
    assert(m_constraints);
    const auto &con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State &s1, const State &s2) {
    assert(m_constraints);
    const auto &con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  size_t m_agentIdx;
  const Constraints *m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
  double m_minAgentCenterDistance;
  double m_minAgentCenterDistanceSquared;
};

#endif
