#ifndef ECBS_PLANNER__AUTHORITY_LEDGER_HPP_
#define ECBS_PLANNER__AUTHORITY_LEDGER_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ecbs_planner/route_occurrence.hpp"

namespace ecbs_planner
{

/// Independent of conflict-group lifetime. Groups reference grants; they never
/// own grant deletion. Destination occupancy survives task-leg completion.
struct LedgerGrant
{
  std::string amr_id;
  uint64_t grant_id{0};
  uint64_t proposal_id{0};
  int32_t release_through_index{-1};
  RouteOccurrence from_occurrence;
  RouteOccurrence to_occurrence;
  uint32_t path_revision{0};
  std::string task_id;
  std::string execution_order_id;
  std::string action_id;
  std::vector<std::string> required_conflict_ids;
  uint64_t commitment_version{0};
};

struct AuthorityLedgerEntry
{
  LedgerGrant grant;
  bool destination_occupied{false};
  bool task_leg_complete{false};
  std::vector<std::string> historical_conflict_ids;
  uint64_t authority_version{0};
};

class AuthorityLedger
{
public:
  void put_grant(const LedgerGrant & grant)
  {
    auto & e = by_amr_[grant.amr_id];
    e.grant = grant;
    e.authority_version = grant.commitment_version;
    for (const auto & c : grant.required_conflict_ids) {
      if (std::find(
          e.historical_conflict_ids.begin(),
          e.historical_conflict_ids.end(), c) ==
        e.historical_conflict_ids.end())
      {
        e.historical_conflict_ids.push_back(c);
      }
    }
    grants_by_id_[grant.grant_id] = grant.amr_id;
  }

  bool retire_grant(const std::string & amr_id, uint64_t grant_id)
  {
    const auto it = by_amr_.find(amr_id);
    if (it == by_amr_.end() || it->second.grant.grant_id != grant_id) {
      return false;
    }
    grants_by_id_.erase(grant_id);
    it->second.grant = LedgerGrant{};
    it->second.grant.amr_id = amr_id;
    ++it->second.authority_version;
    return true;
  }

  void set_destination_occupied(const std::string & amr_id, bool occupied)
  {
    by_amr_[amr_id].destination_occupied = occupied;
    ++by_amr_[amr_id].authority_version;
  }

  std::optional<AuthorityLedgerEntry> get(const std::string & amr_id) const
  {
    const auto it = by_amr_.find(amr_id);
    if (it == by_amr_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  const std::unordered_map<std::string, AuthorityLedgerEntry> & all() const
  {
    return by_amr_;
  }

  void note_conflict_retired(const std::string & /*conflict_id*/)
  {
    // Historical refs retained until explicit settle — no grant deletion.
  }

  uint64_t authority_version_for(const std::string & amr_id) const
  {
    const auto it = by_amr_.find(amr_id);
    return it == by_amr_.end() ? 0 : it->second.authority_version;
  }

private:
  std::unordered_map<std::string, AuthorityLedgerEntry> by_amr_;
  std::unordered_map<uint64_t, std::string> grants_by_id_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__AUTHORITY_LEDGER_HPP_
