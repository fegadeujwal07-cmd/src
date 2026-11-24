# Structured report when a newly detected conflict intersects a committed grant.

string amr_id
string conflict_id
uint64 grant_id
string reason
RouteOccurrence conflict_entry
RouteOccurrence grant_end
bool reservation_retained
# Existing stop/cancel/reconcile protocol must be invoked — no silent shrink.
bool invoke_stop_protocol
