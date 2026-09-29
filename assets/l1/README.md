# L1 asset provenance

The `source.lock.json` pins six RoboDojo USDZ/metadata pairs at dataset revision
`43dacb13d3051a9ccd421f4e7e08e4e3eb29dfab`, including per-file SHA-256.

Download with `python3 tools/fetch_l1_assets.py`. The ignored `source/` directory
is identical on Windows and Linux; downloaded data is verified before scene use.
The upstream dataset declares Apache-2.0; its license is retained alongside this
notice. Source: https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo

The original files are immutable. The runtime creates stage overrides only:
normalization of units/axes, centered object frames, explicit mass and friction,
one dynamic body per article, CPU convex-decomposition collision for dynamic
meshes and baked static triangle surfaces for fixtures. Where an SDF mesh has
internal faces, its corresponding visual surface supplies the static collider.
The test-tube rack is enlarged by 20% along X/Y. These are project-specific
benchmark variants, not original RoboDojo scores or real hardware validation.

L1-02/04/06/07/08 use project-authored parameterized geometry matching the Feishu
task requirements. The AssemblingKits source archive was unavailable during
implementation; L1-02 explicitly uses a T-shaped block and recess, and does not
claim to contain ManiSkill meshes. There is no runtime asset fallback.
