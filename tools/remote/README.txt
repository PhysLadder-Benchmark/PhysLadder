Remote operation (optional)

See docs/release/INSTALL.md for simulator/planner setup and model transport.

1. Copy fleet.json to fleet.local.json, which is ignored by Git.
2. Configure host, user, SSH port, project and runtime directories.
3. Install the project and external runtimes on that host.
4. From the local machine, `python tools/remote/fleet.py status` checks jobs.
5. `connect` establishes an authenticated loopback bridge for the local Codex CLI.
6. `start --machine cluster --tasks L2-02_variant01 --max-calls 150 --wall-time-s 18000`
   launches a task with explicit budgets. It uses the configured model account.

The placeholder fleet.json is not a real machine. An explicit
GPT_BENCHMARK_FLEET_CONFIG overrides fleet.local.json. SSH authentication and
bridge tokens are private; never put them in a release archive.

Do not synchronize a running experiment's dependencies. Never kill unrelated
jobs to deploy. This checkout may have additional rules in AGENTS.local.md.
Historical project directories and experiment snapshots must remain intact.
