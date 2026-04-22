#!/usr/bin/env python3
"""
Compare two ROS bags for CONTENT equivalence (not byte equivalence).

Two bags are content-identical iff:
  - Same set of topics
  - Same number of messages per topic
  - For each topic, the sorted (bag_time_nsec, hash(serialized_bytes))
    pairs match element-for-element.

Physical layout, chunk ordering, and connection metadata are ignored.
Uses rosbag raw=True so we compare the exact serialized bytes written.

Optimizations:
  - Loads both bags in parallel (one subprocess per bag)
  - Uses xxhash (~10x faster than md5); install with `pip install xxhash`
    Falls back to hashlib.blake2b-8 if xxhash is unavailable
  - Stores per-topic (time, hash) as numpy arrays; compare is vectorized

Usage:
    python3 compare_bags.py <bag_a> <bag_b>
"""

import sys
import time
from collections import defaultdict
from multiprocessing import Pool

import numpy as np
import rosbag

try:
    import xxhash
    def _hash(data):
        return xxhash.xxh3_64_intdigest(data)
    _HASH_NAME = "xxh3_64"
except ImportError:
    import hashlib
    def _hash(data):
        return int.from_bytes(hashlib.blake2b(data, digest_size=8).digest(), 'big')
    _HASH_NAME = "blake2b-8 (install `xxhash` for ~10x speedup)"


def hash_bag(path):
    """Return {topic: (times_ns: int64[N], hashes: uint64[N])}, sorted by time."""
    t0 = time.time()
    per_times = defaultdict(list)
    per_hashes = defaultdict(list)
    n = 0
    with rosbag.Bag(path, "r") as bag:
        total = bag.get_message_count()
        for topic, raw, t in bag.read_messages(raw=True):
            per_times[topic].append(t.to_nsec())
            per_hashes[topic].append(_hash(raw[1]))
            n += 1
            if n % 50000 == 0:
                dt = time.time() - t0
                rate = n / dt
                eta = (total - n) / rate if rate > 0 else 0
                print(f"  [{path}] {n}/{total} ({100 * n / total:.0f}%) "
                      f"{rate:.0f} msg/s  ETA {eta:.0f}s", flush=True)

    result = {}
    for topic in per_times:
        times = np.asarray(per_times[topic], dtype=np.int64)
        hashes = np.asarray(per_hashes[topic], dtype=np.uint64)
        order = np.argsort(times, kind='stable')
        result[topic] = (times[order], hashes[order])
    dt = time.time() - t0
    print(f"  [{path}] done: {n} msgs in {dt:.1f}s ({n / dt:.0f} msg/s)",
          flush=True)
    return result


def compare(path_a, path_b):
    print(f"Hash: {_HASH_NAME}")
    print("Loading both bags in parallel...", flush=True)
    t0 = time.time()
    with Pool(2) as pool:
        a, b = pool.map(hash_bag, [path_a, path_b])
    print(f"Total wall time: {time.time() - t0:.1f}s\n")

    topics_a, topics_b = set(a), set(b)
    ok = True

    only_a = topics_a - topics_b
    only_b = topics_b - topics_a
    if only_a:
        ok = False
        print(f"[DIFF] topics only in A: {sorted(only_a)}")
    if only_b:
        ok = False
        print(f"[DIFF] topics only in B: {sorted(only_b)}")

    for topic in sorted(topics_a & topics_b):
        ta, ha = a[topic]
        tb, hb = b[topic]
        if len(ta) != len(tb):
            print(f"[DIFF] {topic}: {len(ta)} in A vs {len(tb)} in B")
            ok = False
            continue
        t_ok = np.array_equal(ta, tb)
        h_ok = np.array_equal(ha, hb)
        if t_ok and h_ok:
            print(f"[OK]   {topic}: {len(ta)} messages identical")
        else:
            ok = False
            diff = (ta != tb) | (ha != hb)
            idx = np.where(diff)[0]
            print(f"[DIFF] {topic}: {len(idx)}/{len(ta)} mismatched")
            for i in idx[:5]:
                print(f"         idx {i}: A=(t={ta[i]}, h={ha[i]:016x})  "
                      f"B=(t={tb[i]}, h={hb[i]:016x})")
            if len(idx) > 5:
                print(f"         ... +{len(idx) - 5} more")

    print()
    if ok:
        print("== RESULT: bags are content-identical ==")
        return 0
    print("== RESULT: bags DIFFER ==")
    return 1


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(compare(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
