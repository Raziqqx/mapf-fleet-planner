#pragma once
// Converts planner types into the validator's own types. This is the only
// place the two components meet; the validator itself knows nothing of mapf::.

#include "mapf/planner.hpp"
#include "mapf_validator/validator.hpp"

namespace mapf {

mapf_validator::Problem to_validator_problem(const Instance& instance);
std::vector<std::vector<mapf_validator::Pos>> to_validator_plans(const Grid& grid,
                                                                 const std::vector<Path>& paths);
// Validates a planner result; the report's sum_of_costs is computed independently.
mapf_validator::Report validate_result(const Instance& instance, const Result& result);

}  // namespace mapf
