# fetch-mujoco

The engine's body physics (`crates/euc-physics`) runs on [MuJoCo](https://mujoco.org) (Apache-2.0)
through the `mujoco-rs` crate, which links the official MuJoCo release dynamically. This tool
downloads that release for your OS from DeepMind's GitHub releases, checks it against the sha256
pinned in `fetch.py`, and unpacks it into `.cache/mujoco/` in the repository.

```sh
python3 tools/fetch-mujoco/fetch.py              # once per clone (or worktree)
cargo build                                      # finds it through .cargo/config.toml
```

| OS | Asset | Run-time lookup |
|---|---|---|
| Linux x86_64, aarch64 | `.tar.gz` | rpath to `.cache/mujoco/mujoco-3.9.0/lib` (set by the crates' build scripts) |
| macOS (universal) | `.dmg` (opened read-only with `hdiutil`) | rpath; `lib/` holds `mujoco.framework` and a `libmujoco.dylib` link into it |
| Windows x86_64 | `.zip` | `mujoco.dll` in `bin/` must be on `PATH` (`--print-env` shows how) |

- `--dest DIR` puts the cache elsewhere; then set `MUJOCO_DYNAMIC_LINK_DIR` to its `lib/` folder
  (`--print-env` prints the line). An environment variable always wins over `.cargo/config.toml`.
- `--github` (CI) exports `MUJOCO_DYNAMIC_LINK_DIR` and, on Windows, adds the DLL folder to `PATH`.
- Running again does nothing when the pinned release is already unpacked.

The version must match the `+mj-` suffix of the `mujoco-rs` version in the workspace `Cargo.toml`
(`5.0.0+mj-3.9.0` today); `test_fetch.py` checks both pins. To move to a new MuJoCo, bump both, and
copy the new sha256 values from the `.sha256` files DeepMind publishes next to each asset.

```sh
python3 -m unittest discover -s tools/fetch-mujoco -p 'test_*.py'
```
