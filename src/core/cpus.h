#pragma once
// How many CPUs this process can actually use.
//
// std::thread::hardware_concurrency() answers for the machine, not for the container: an ECS
// task given 2 vCPU on a 16-core host sees 16, and a thread pool sized from that runs eight
// times wider than its CPU quota. Past the quota the kernel throttles the whole cgroup for the
// rest of each scheduling period — the physics tick included — so an oversized pool does not
// just waste CPU, it stalls the world. Sizing from this instead keeps a pool inside what the
// box will really run.
#include <string>

namespace skysim::core {

// The smallest of: the online CPU count, the scheduler affinity mask, and the cgroup CPU quota
// (cgroup v2 cpu.max, or v1 cpu.cfs_quota_us / cpu.cfs_period_us) of this process's cgroup and
// every ancestor visible from here. A fractional quota rounds down — 1.5 CPUs of quota keeps
// one thread busy without throttling, two threads cannot — and the result is at least 1.
int available_cpus();

// --- pieces, exposed for the tests ---------------------------------------------------------

// CPUs a cgroup v2 cpu.max line allows ("200000 100000" -> 2.0), or 0 for no limit ("max ...")
// and for anything unparseable.
double cgroup_v2_quota_cpus(const std::string &cpu_max);

// The same from cgroup v1's two files; a quota of -1 (or any non-positive value) is no limit.
double cgroup_v1_quota_cpus(long long quota_us, long long period_us);

// The tightest CPU quota on the path to this process's cgroup, 0 when none is set.
// `cgroup_root` is where the cgroup filesystem is mounted (/sys/fs/cgroup), and `proc_cgroup`
// the contents of /proc/self/cgroup, which name the process's cgroup under it.
double cgroup_quota_cpus(const std::string &cgroup_root, const std::string &proc_cgroup);

// The default number of pilot-camera render threads for `cpus` usable CPUs: one per CPU, at
// most four (enough for any frame size worth streaming), at least one. No CPU needs leaving
// free for the tick: the render threads run at nice +10, and measured on a 2-CPU quota with
// three watched vehicles flying the tick held 400 Hz with two render threads saturating it,
// where the old five un-niced threads had dropped it to 378 Hz. A spare CPU would only halve
// the frame rate (8.8 -> 4.7 fps each).
int default_render_threads(int cpus);

} // namespace skysim::core
