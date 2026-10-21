# Repository maintenance

Read `AGENTS.local.md` if present. It contains private deployment rules and takes
precedence for synchronization. Never commit local rules, credentials, or host
configuration.

- Original and variant experiments are independent configurations sharing code.
  Public `_variant01` aliases map to legacy `_v3` identities. Keep historical IDs.
- Preserve prompts, scoring, physics, cameras, and initial layouts during
  infrastructure changes. Check `tools/release/check_task_contract.py` and
  `tools/verify_runtime_assets.py`; never refresh locks just to hide a regression.
- Offline tests must not call a model. Startup checks do not prove task success.
- Check running processes before deploying. Never interrupt experiments to sync
  or overwrite another project or historical run.
- `tools/release/package.py` prepares source and complete hash-locked assets,
  excluding local state and Git history.
