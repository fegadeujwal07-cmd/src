#ifndef ECBS_PLANNER__WAIT_REASON_HPP_
#define ECBS_PLANNER__WAIT_REASON_HPP_

#include <string>

namespace ecbs_planner
{

/// EvaluatePath.Response.wait_reason values (L1 classification for TM).
inline constexpr const char * kWaitReasonNone = "";
inline constexpr const char * kWaitReasonOwnership = "ownership";
/// Head-on: challenger at start, owner at last granted base.
inline constexpr const char * kWaitReasonOpposite = "opposite";
/// Crossing (+): priority arbitration in TM.
inline constexpr const char * kWaitReasonPlus = "plus";
/// T / inverted-T: one continues through, other branches (path easiness).
inline constexpr const char * kWaitReasonTJunction = "t_junction";
/// Merge: two approaches join then travel same way (clearer-first, else priority).
inline constexpr const char * kWaitReasonMerge = "merge";
/// Same-way: trailing robot waits.
inline constexpr const char * kWaitReasonBackToBack = "back_to_back";
/// Legacy alias — TM treats like plus (priority arbitrate).
inline constexpr const char * kWaitReasonCorridor = "corridor";
inline constexpr const char * kWaitReasonJunction = "junction";

/// WaitDependency.topology values (per-blocker geometry).
inline constexpr const char * kTopoOwnership = "ownership";
inline constexpr const char * kTopoOpposite = "opposite";
inline constexpr const char * kTopoSameWay = "same_way";
inline constexpr const char * kTopoMerge = "merge";
inline constexpr const char * kTopoCross = "cross";
inline constexpr const char * kTopoJunction = "junction";

/// Response.wait_reason for one dependency topology.
inline std::string wait_reason_from_topology(const std::string & topo)
{
  if (topo == kTopoOwnership) {return kWaitReasonOwnership;}
  if (topo == kTopoOpposite) {return kWaitReasonOpposite;}
  if (topo == kTopoSameWay) {return kWaitReasonBackToBack;}
  if (topo == kTopoMerge) {return kWaitReasonMerge;}
  if (topo == kTopoCross) {return kWaitReasonPlus;}
  if (topo == kTopoJunction) {return kWaitReasonJunction;}
  return kWaitReasonNone;
}

/// Strength: ownership > opposite > plus|t|merge|b2b|corridor > junction > none.
inline int wait_reason_rank(const std::string & reason)
{
  if (reason == kWaitReasonOwnership) {
    return 5;
  }
  if (reason == kWaitReasonOpposite) {
    return 4;
  }
  if (reason == kWaitReasonPlus ||
    reason == kWaitReasonTJunction ||
    reason == kWaitReasonMerge ||
    reason == kWaitReasonBackToBack ||
    reason == kWaitReasonCorridor)
  {
    return 3;
  }
  if (reason == kWaitReasonJunction) {
    return 1;
  }
  return 0;
}

/// Keep the stronger (higher-rank) wait_reason already on \p current.
inline void strengthen_wait_reason(std::string & current, const std::string & candidate)
{
  if (wait_reason_rank(candidate) > wait_reason_rank(current)) {
    current = candidate;
  }
}

/// Infer wait_reason from an evaluate message when not set explicitly.
inline std::string infer_wait_reason_from_message(const std::string & message)
{
  // Base-end waiter cross: ECBS defers to TM — never ownership HardHold.
  if (message.find("traffic manager decides") != std::string::npos ||
    message.find("at base end") != std::string::npos)
  {
    return kWaitReasonCorridor;
  }
  if (message.find("base reserved") != std::string::npos ||
    message.find("robot standing at") != std::string::npos ||
    message.find("no register needed") != std::string::npos ||
    message.find("outside reserved base") != std::string::npos ||
    message.find("foreign base") != std::string::npos)
  {
    return kWaitReasonOwnership;
  }
  if (message.find("opposite") != std::string::npos ||
    message.find("head-on") != std::string::npos ||
    message.find("head on") != std::string::npos)
  {
    return kWaitReasonOpposite;
  }
  if (message.find("back-to-back") != std::string::npos ||
    message.find("back to back") != std::string::npos ||
    message.find("trailing") != std::string::npos)
  {
    return kWaitReasonBackToBack;
  }
  if (message.find("merge") != std::string::npos &&
    message.find("emergency") == std::string::npos)
  {
    return kWaitReasonMerge;
  }
  if (message.find("t_junction") != std::string::npos ||
    message.find("T-junction") != std::string::npos ||
    message.find("easiness") != std::string::npos ||
    message.find("inverted-T") != std::string::npos)
  {
    return kWaitReasonTJunction;
  }
  if (message.find("plus") != std::string::npos ||
    message.find("crossing") != std::string::npos)
  {
    return kWaitReasonPlus;
  }
  if (message.find("junction") != std::string::npos ||
    message.find(" before junction ") != std::string::npos ||
    message.find("T/+") != std::string::npos ||
    message.find("base cut before") != std::string::npos)
  {
    return kWaitReasonJunction;
  }
  if (message.find("WAIT") != std::string::npos ||
    message.find("holding for") != std::string::npos ||
    message.find("conflict") != std::string::npos)
  {
    return kWaitReasonCorridor;
  }
  return kWaitReasonNone;
}

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__WAIT_REASON_HPP_
