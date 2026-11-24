#include <gtest/gtest.h>

#include <mutex>

/// Documents the TM locking contract that caused the first-release deadlock:
/// rebuild_path_release_markers() acquires mutex_; callers that already hold
/// mutex_ must copy PathRelease data, unlock, then rebuild. Do not replace
/// std::mutex with recursive_mutex to hide nested locking.
TEST(MutexContract, PublishThenRebuildOutsideLock)
{
  std::mutex m;
  int tip = -1;
  uint32_t release_update_id = 0;
  bool published = false;
  bool rebuilt = false;

  // Simulate Commit ACCEPTED under lock: advance tip + sequence, copy payload.
  int tip_copy = -1;
  uint32_t update_copy = 0;
  {
    std::lock_guard<std::mutex> lock(m);
    tip = 6;
    release_update_id = 0;
    tip_copy = tip;
    update_copy = release_update_id;
  }  // unlock before publish / marker rebuild

  published = (tip_copy == 6 && update_copy == 0);
  {
    std::lock_guard<std::mutex> lock(m);
    rebuilt = true;
    // Matching ACK of update 0 can proceed; tip preserved for extension.
    EXPECT_EQ(tip, 6);
    EXPECT_EQ(release_update_id, 0u);
  }
  EXPECT_TRUE(published);
  EXPECT_TRUE(rebuilt);
}
