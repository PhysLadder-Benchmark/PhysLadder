# L3-04 initial-state translations

The original task retains its delivered initial state. The default
`L3-04_variant01` (`L3-04_v3`) restores the original aligned two-book stack and
translates both books, the starting rectangle, and the target rectangle by
`-0.10 m` along world X, placing the stack in the middle of the fixed overview.

The optional task setting `stacked_book_offset_x_m` accepts a finite offset up
to 30 cm in either direction; any newly chosen layout needs its own tabletop and
reachability checks. It is applied once, before physics initialization,
on the USD session layer. Source USD files and historical runs remain intact.
The rendered rectangle transforms, reset poses, and scoring polygons receive
the same translation. Heights, dimensions, materials, cameras, robot initial
states, and relative book/rectangle orientations are preserved from the original.

This changes placement only. The original task still requires both books in
the target; the existing variant still judges containment by the lower book.
Both require a full 360-degree stack rotation during tabletop transfer, with
the existing support and no-grasp/no-lift constraints.

In an installed Isaac environment, render the centered variant without a
model request:

```bash
python tools/release/render_initial.py L3-04_variant01 --gpu 0 --reset-only \
  --output build/stacked_book_center_preview
```

Startup, table-clearance, and joint-limited reach checks establish initial
placement feasibility, not successful execution of the complete manipulation.

Additional placements can be reviewed with `--config` and an ignored
`*.local.toml` override; they do not create or change a registered task.
