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

Open `http://localhost:8000`. The PhysLadder page includes an interactive particle constellation, a compact sixteen-task
simulation/real-robot video montage, individual paired excerpts, and a downloadable
GIF. The hero uses the paper’s complete title and subtitle. An interactive ARX R5
gripper is drawn from 7,400 points and contour edges sampled from the pinned
manufacturer CAD (`link6`–`link8`), with drag/keyboard rotation, hover connections
and a normalized opening control. It shares the page’s motion preference, pauses
when offscreen, and includes a static SVG fallback. Regenerate the presentation
assets with `python tools/site/build_gripper_art.py`; runtime assets remain unchanged.
Source hashes and illustration boundaries are recorded in
`site/media/gripper/sources.json`. Task filters, original/variant comparison, three camera views and copy buttons
work without a backend. The sim–real matching section adds nine paired views from paper
Figure 5: workspace, initial nut engagement, ARX R5 embodiment, three camera
observations, measured marker length and jelly mass. The title-page institution
marks appear as author affiliations below the hero. Motion pauses outside the viewport or in background tabs;
reduced-motion preferences disable automatic animation and playback. The task catalogue is in
`tasks.json`; `tasks.js` also allows opening the page directly from disk.

The manual `.github/workflows/pages.yml` publishes only `site/` using GitHub
Pages. Select **GitHub Actions** in the repository's Pages settings, then run
**Publish project page** after pushing the reviewed release. The project page is hosted at
<https://physladder-benchmark.github.io/PhysLadder/>; repository and configuration
links target `PhysLadder-Benchmark/PhysLadder`. No custom domain is required.
No empty paper/PDF link is shown. The title uses the supplied paper title; author,
venue and paper URL metadata have not been invented.

Design references: [RoboCasa](https://robocasa.ai/),
[BEHAVIOR](https://behavior.stanford.edu/index.html), and
[Gemini Robotics](https://deepmind.google/models/gemini-robotics/).
The layout, particle graphics and code are original. Project-owned experiment
recordings supply the looping media. Institution marks and paired matching images
are reproduced from the authors’ supplied paper, with provenance in
`site/media/affiliations/sources.json` and `site/media/alignment/sources.json`.
Institution marks identify author affiliations, not institutional endorsement.
Camera examples preserve the paper’s timing caveats; no exact intrinsic
calibration is claimed. AI-generated appearance composites from Figure 5(b3)
are excluded. The hardware is identified as ARX R5 (方舟).
`site/media/loops/sources.json` records the original filenames, SHA-256 hashes,
selected eight-second windows and overview-camera crops. These are presentation
excerpts, with no additional speed changes; the two domains have independent
timelines and outcome labels refer to full trials. The downloadable GIF and MP4
montage combine all thirty-two excerpts. The overview uses four task columns on
desktop and two on mobile; selecting a task opens its paired clips. Rebuild the
montages with `python tools/site/build_film_montage.py --ffmpeg /path/to/ffmpeg`. The marble-pouring simulation excerpt uses
the first trial (12/12 marbles received), replacing the second trial with spillage.
Source recordings are kept in the materials
document; temporary local downloads are removed after derivative verification.

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
