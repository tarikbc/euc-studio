#!/usr/bin/env python3
"""Download the MuJoCo native library that the engine links against, and check it.

The engine's physics (crates/euc-physics) uses MuJoCo through the `mujoco-rs` crate, which links the official
MuJoCo release dynamically. This script downloads that release for the current OS from DeepMind's GitHub releases,
checks it against the sha256 pinned below, and unpacks it into a cache folder:

    <dest>/mujoco-<version>/lib/     what MUJOCO_DYNAMIC_LINK_DIR points at
        libmujoco.so[.x.y.z]          Linux
        mujoco.framework/ + libmujoco.dylib (a link into the framework)   macOS
        mujoco.lib                    Windows (the DLL is in ../bin/)

The default <dest> is `.cache/mujoco` in the repository. `.cargo/config.toml` points MUJOCO_DYNAMIC_LINK_DIR there,
so after one run of this script `cargo build` finds the library with no further setup. Set MUJOCO_DYNAMIC_LINK_DIR
yourself to use another copy.

Python 3.11+ standard library only. macOS needs `hdiutil` (part of the OS) to read the disk image.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path, PurePosixPath

VERSION = "3.9.0"  # must match the `+mj-` suffix of the mujoco-rs version in Cargo.toml
BASE_URL = f"https://github.com/google-deepmind/mujoco/releases/download/{VERSION}"

# Release assets and their sha256, as DeepMind publishes them next to each asset (<asset>.sha256).
ASSETS: dict[tuple[str, str], tuple[str, str]] = {
    ("linux", "x86_64"): (
        f"mujoco-{VERSION}-linux-x86_64.tar.gz",
        "d11f281540d0d1844e2923bf43b6fff5ad186ec55927a8dae0eb26b9e579eed2",
    ),
    ("linux", "aarch64"): (
        f"mujoco-{VERSION}-linux-aarch64.tar.gz",
        "47f295e1c7bef45a6f0d26108b275865d3097012bd2b02f5938d7adfade0d8da",
    ),
    ("macos", "universal2"): (
        f"mujoco-{VERSION}-macos-universal2.dmg",
        "a1229a5d068f52272a329c0962090b9cd0639eff04db48d08ecd2deaaf4a8780",
    ),
    ("windows", "x86_64"): (
        f"mujoco-{VERSION}-windows-x86_64.zip",
        "544f44a8a7df3e94648a7eaf41500f4456eb59f9f01df3ec2cfb03bdbf5c2bb9",
    ),
}

REPO = Path(__file__).resolve().parents[2]
DEFAULT_DEST = REPO / ".cache" / "mujoco"
STAMP = ".euc-sha256"  # written last: a folder without it is an interrupted unpack and is replaced


def host() -> tuple[str, str]:
    """The (os, arch) key of ASSETS for this machine."""
    system = platform.system()
    machine = platform.machine().lower()
    if system == "Darwin":
        return "macos", "universal2"
    if system == "Linux":
        return "linux", {"amd64": "x86_64", "arm64": "aarch64"}.get(machine, machine)
    if system == "Windows":
        return "windows", {"amd64": "x86_64"}.get(machine, machine)
    return system.lower(), machine


def library_file(os_name: str, lib_dir: Path) -> Path:
    """The file mujoco-rs's build script looks for in MUJOCO_DYNAMIC_LINK_DIR."""
    return lib_dir / {"linux": "libmujoco.so", "macos": "libmujoco.dylib", "windows": "mujoco.lib"}[os_name]


def runtime_dir(os_name: str, root: Path) -> Path:
    """The folder the loader must search at run time (rpath on Unix, PATH on Windows)."""
    return root / ("bin" if os_name == "windows" else "lib")


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url: str, to: Path) -> None:
    print(f"downloading {url}", file=sys.stderr)
    with urllib.request.urlopen(url, timeout=60) as response, to.open("wb") as out:  # noqa: S310 (fixed https URL)
        shutil.copyfileobj(response, out, 1 << 20)


def safe_member(name: str) -> PurePosixPath:
    """An archive member path, refused if it could escape the folder it is unpacked into."""
    path = PurePosixPath(name.replace("\\", "/"))
    if path.is_absolute() or ".." in path.parts or (path.parts and ":" in path.parts[0]):
        raise SystemExit(f"error: unsafe path in archive: {name!r}")
    return path


def single_top(names: list[PurePosixPath]) -> str | None:
    """The one top-level folder every member is inside, if there is one."""
    tops = {p.parts[0] for p in names if p.parts}
    if len(tops) == 1 and any(len(p.parts) > 1 for p in names):
        return tops.pop()
    return None


def unpack_tar(archive: Path, into: Path) -> None:
    with tarfile.open(archive) as tar:
        members = tar.getmembers()
        top = single_top([safe_member(m.name) for m in members])
        tar.extractall(into, filter="data")  # the data filter also refuses links that leave `into`
    if top:
        hoist(into / top, into)


def unpack_zip(archive: Path, into: Path) -> None:
    # The Windows release names its members with backslashes, so extract by hand with the
    # separators normalised, the same on every OS.
    with zipfile.ZipFile(archive) as z:
        members = [(info, safe_member(info.filename)) for info in z.infolist()]
        top = single_top([path for _, path in members])
        for info, path in members:
            target = into.joinpath(*path.parts)
            if info.is_dir() or info.filename.endswith(("/", "\\")):
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with z.open(info) as src, target.open("wb") as dst:
                shutil.copyfileobj(src, dst, 1 << 20)
    if top:
        hoist(into / top, into)


def unpack_dmg(archive: Path, into: Path) -> None:
    """Copy mujoco.framework out of the disk image into lib/, plus a libmujoco.dylib link for the linker."""
    with tempfile.TemporaryDirectory(prefix="mujoco-dmg-") as mount:
        hdiutil("attach", "-nobrowse", "-readonly", "-noautoopen", "-mountpoint", mount, str(archive))
        try:
            lib = into / "lib"
            lib.mkdir(parents=True)
            # ditto keeps the framework's symlinks and code signature intact.
            subprocess.run(["ditto", f"{mount}/mujoco.framework", str(lib / "mujoco.framework")], check=True)
            shutil.copy2(f"{mount}/LICENSE", into / "LICENSE")
            shutil.copy2(f"{mount}/THIRD_PARTY_NOTICES", into / "THIRD_PARTY_NOTICES")
        finally:
            hdiutil("detach", mount)
    # The library's install name is @rpath/mujoco.framework/Versions/A/libmujoco.<v>.dylib, so an rpath of lib/
    # finds it at run time; the linker only needs a plain libmujoco.dylib to link against.
    (lib / "libmujoco.dylib").symlink_to(f"mujoco.framework/Versions/A/libmujoco.{VERSION}.dylib")


def hdiutil(*args: str) -> None:
    """Run hdiutil quietly (it warns that attach and detach are deprecated); show its output only on failure."""
    done = subprocess.run(["hdiutil", *args], capture_output=True, text=True, check=False)
    if done.returncode != 0:
        raise SystemExit(f"error: hdiutil {args[0]} failed:\n{done.stdout}{done.stderr}")


def hoist(src: Path, dst: Path) -> None:
    """Move the contents of `src` up into `dst` and remove `src`."""
    for child in list(src.iterdir()):
        child.rename(dst / child.name)
    src.rmdir()


def fetch(dest: Path) -> tuple[str, Path]:
    """Make sure the pinned release is unpacked under `dest`; return (os, root folder)."""
    key = host()
    if key not in ASSETS:
        raise SystemExit(
            f"error: no MuJoCo {VERSION} build for {key[0]} {key[1]}. "
            "Build MuJoCo yourself and set MUJOCO_DYNAMIC_LINK_DIR to its lib/ folder."
        )
    os_name = key[0]
    asset, pin = ASSETS[key]
    root = dest / f"mujoco-{VERSION}"
    stamp = root / STAMP

    if stamp.is_file() and stamp.read_text().strip() == pin and library_file(os_name, root / "lib").exists():
        print(f"MuJoCo {VERSION} already in {root}", file=sys.stderr)
        return os_name, root

    dest.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mujoco-", dir=dest) as tmp:
        archive = Path(tmp) / asset
        download(f"{BASE_URL}/{asset}", archive)
        got = sha256_of(archive)
        if got != pin:
            raise SystemExit(f"error: sha256 mismatch for {asset}\n  expected {pin}\n  got      {got}")
        staged = Path(tmp) / "unpacked"
        staged.mkdir()
        if asset.endswith(".tar.gz"):
            unpack_tar(archive, staged)
        elif asset.endswith(".zip"):
            unpack_zip(archive, staged)
        else:
            unpack_dmg(archive, staged)
        if not library_file(os_name, staged / "lib").exists():
            raise SystemExit(f"error: {asset} has no {library_file(os_name, Path('lib')).as_posix()}")
        (staged / STAMP).write_text(pin + "\n")
        if root.exists():
            shutil.rmtree(root)
        staged.rename(root)
    print(f"MuJoCo {VERSION} unpacked into {root}", file=sys.stderr)
    return os_name, root


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dest", type=Path, default=DEFAULT_DEST, help=f"cache folder (default: {DEFAULT_DEST})")
    out = parser.add_mutually_exclusive_group()
    out.add_argument("--print-env", action="store_true", help="print shell lines that point the build at the library")
    out.add_argument(
        "--github",
        action="store_true",
        help="in GitHub Actions: export MUJOCO_DYNAMIC_LINK_DIR and put the DLL folder on PATH (Windows)",
    )
    args = parser.parse_args(argv)

    os_name, root = fetch(args.dest.resolve())
    lib = root / "lib"
    if args.print_env:
        if os_name == "windows":
            print(f'$env:MUJOCO_DYNAMIC_LINK_DIR = "{lib}"')
            print(f'$env:PATH = "{runtime_dir(os_name, root)};$env:PATH"')
        else:
            print(f"export MUJOCO_DYNAMIC_LINK_DIR='{lib}'")
    elif args.github:
        with open(os.environ["GITHUB_ENV"], "a", encoding="utf-8") as env:
            env.write(f"MUJOCO_DYNAMIC_LINK_DIR={lib}\n")
        if os_name == "windows":
            with open(os.environ["GITHUB_PATH"], "a", encoding="utf-8") as path:
                path.write(f"{runtime_dir(os_name, root)}\n")
    else:
        print(lib)
    return 0


if __name__ == "__main__":
    sys.exit(main())
