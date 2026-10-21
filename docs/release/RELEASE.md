# Source, assets and project page

## Deliverable layout

`python tools/release/package.py --output dist` builds:

- `r5-benchmark-source.tar.gz`: maintained code, all task configurations, tests,
  release documentation, optional deployment skill, site and the runtime asset lock. No `.git` history,
  credentials, host-local config, environments, runs, reports or caches.
- `r5-benchmark-assets.tar.gz`: every file in `assets/runtime.lock.json`, including
  all references, source layers, textures and upstream notices.
- `release.json`: SHA-256 and size of both archives, plus per-source-file hashes.

Extract source into a new directory. Install its matching asset archive with
`tools/release/install_assets.py`; the installer validates all paths, sizes and
hashes before replacing runtime inputs. Do not use an older Tuojing package as
a substitute. Asset installation should occur while that checkout is idle.

The normal working tree still keeps all runtime assets. `.gitignore` excludes
outputs and local configuration, not required task inputs. This release does not
require Git LFS. For a smaller source-only Git repository, attach the complete
asset archive to the SAME GitHub release and document its checksum; never omit
assets without a working restoration path. The archive installer is that path.

## Fresh-checkout acceptance

1. Extract the source archive into a clean directory outside the development tree.
2. Restore its matching assets and verify `verify_runtime_assets.py --usd`.
3. Install `.[test]`, run CPU tests, task-contract check and offline dry runs.
4. Configure external Isaac/cuRobo installations; capture all 32 initial states.
5. Compare task fingerprints and asset hashes with the frozen pre-release baseline.
6. Archive verification output separately; no model call is needed for these checks.

Initial-state validation is not a claim that a model can solve every task.
A paper result reproduction additionally requires the chosen model access and
full episodes under the stated budgets.

## GitHub Pages

`site/` is buildless HTML/CSS/JavaScript. Its actual simulator images are local
WebP derivatives; source assets and simulation rendering remain untouched.

```bash
python -m http.server 8000 --directory site
```

Open `http://localhost:8000`. Task filters, original/variant comparison, three
camera views and copy buttons work without a backend. The task catalogue is in
`tasks.json`; `tasks.js` also allows opening the page directly from disk.

The manual `.github/workflows/pages.yml` publishes only `site/` using GitHub
Pages. Select **GitHub Actions** in the repository's Pages settings, then run
**Publish project page** after pushing the reviewed release. Repository/code
links target `TuojingAI/GPT_Benchmark`; update them if publishing a different repo.
No empty paper/PDF link is shown. The title uses the supplied paper title; author,
venue and paper URL metadata have not been invented.

Design references: [RoboCasa](https://robocasa.ai/),
[BEHAVIOR](https://behavior.stanford.edu/index.html), and
[Gemini Robotics](https://deepmind.google/models/gemini-robotics/).
The layout and code are original; no images, logos or result claims were copied.

## Licenses and provenance

Team-owned code is Apache-2.0, approved by the project owner. On 2026-10-21 the
owner also confirmed public redistribution permission for all supplied assets.
Third-party notices remain in place; `NOTICE` and `asset_permissions.json` keep
that confirmation distinct from each upstream license. Isaac and cuRobo remain
external dependencies. Historical local paths in provenance metadata are source
lineage, not runtime installation requirements.

Before public push, review the prepared source rather than pushing an old Git
history that may contain private material. `package.py` never copies Git history.
Paper author list, permanent paper link and final result tables can be supplied
later without altering task definitions.
