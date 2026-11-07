# Validation

Run from the project root:

```powershell
python tests\run_tests.py
python tests\run_clock_tests.py
```

Host tests use a native C++17 compiler (MSVC on Windows, Clang/GCC on POSIX);
executables are built in automatically removed temporary directories.
`--compiler` and `--vcvars` can select an existing toolchain.

## Automated coverage

* Real IPv4/IPv6 codec and PTP parser: all five message types, ports, groups,
  length/truncation, IP/UDP checksums, IPv6 mandatory checksum, forbidden
  fragmentation/extension headers, malformed fields, unsigned-only TLV
  rejection, wrong family/group, Ethernet padding, source-IP changes.
* Timestamp arithmetic/overflow, MAC validation and collisions, restricted
  ranked/pinned selection, expiry, replay/duplicate rejection, out-of-order
  Sync/Follow_Up, requester and sequence matching, fixed-point corrections,
  E2E offset/path delay, measured Sync rate and servo behavior.
* Real clock owner with deterministic SDK/clock/mutex stubs, for both target
  settings branches: startup, lock/loss, stale metadata, generation barriers,
  learned frequency, failed hardware operations, bridge-read preemption and
  read-only snapshots.
* TAI/UTC/local conversions and DST ambiguity handling. Windows CRT does not
  implement full POSIX TZ rule syntax; the runner explicitly reports that
  limitation. Linux CI additionally runs full explicit DST-rule tests.

The inherited core retains security-provenance consistency checks, but there
is no crypto implementation, secret, signing or authentication in this project.
The UDP parser rejects all trailing TLVs, including authentication.

CI builds `pre_v3`, `v3_0`, `v3_1`, and `s31`, each with IPv4 and IPv6 timing.
This catches SDK/board compilation errors, not real packet timing.

## Required physical acceptance

For each supported board, run against a qualified ESP32TimeServer on the same
untagged LAN, separately in IPv4 and IPv6 mode:

1. Verify the startup name/version, actual chip revision, exact active MAC,
   hostname and negotiated link speed. Verify DHCP hostname option 12 with a
   capture for IPv4; do not assume DNS registration.
2. Verify blank MAC override uses the native MAC, the example override sets
   that exact Ethernet MAC, and malformed/non-unicast overrides stop startup.
   Never deploy the same override on more than one device.
3. Capture IGMP/MLD membership and UDP 319/320 to `224.0.1.129` / `ff02::181`.
   Check two-step Sync/Follow_Up and multicast Delay_Req/Delay_Resp requester
   identity/sequence, checksums and TX TTL/hop limit 1.
4. Verify IPv6 timing acquires on link-local addresses with no DHCP/router;
   verify tentative/duplicate IPv6 addresses cannot qualify timing.
5. Observe acquisition through at least eight good E2E measurements, the
   one-time first-lock banner and continued per-second TAI/UTC/local/status.
   Measure actual offset/jitter externally; a printed LOCKED threshold is
   not an independently measured accuracy guarantee.
6. Remove Ethernet, renew/change the DHCP address, restart the master, suppress
   Sync or Delay_Resp, and remove GNSS qualification. Verify lock loss is
   explicit, no stale exchange survives a generation/network transition,
   and automatic reacquisition occurs when qualified input returns.
7. Test a snooping switch, duplicate MACs, wrong domain, malformed packets,
   checksum errors, queue pressure and a better/worse master appearing.
   Confirm unrelated DHCP/ARP/ND traffic continues to reach lwIP.
8. Check local DST transitions and changed UTC offsets. Leap announcements
   must prevent lock during ambiguity, not silently apply a guessed offset.

Socket joins, DHCP/ND recovery, real RX/TX timestamp availability, reporter
scheduling and physical timing accuracy cannot be verified by the host stubs.
No physical acceptance is claimed until these tests run on actual hardware.
