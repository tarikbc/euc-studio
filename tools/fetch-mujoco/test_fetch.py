"""Tests for fetch.py: archive handling and the pins (no network)."""

from __future__ import annotations

import io
import re
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path, PurePosixPath

import fetch


def tar_bytes(files: dict[str, bytes]) -> bytes:
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode="w:gz") as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return out.getvalue()


class ArchiveTests(unittest.TestCase):
    def test_tar_with_a_top_folder_is_hoisted(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / "a.tar.gz"
            archive.write_bytes(tar_bytes({"mujoco-9.9.9/lib/libmujoco.so": b"x", "mujoco-9.9.9/LICENSE": b"l"}))
            into = Path(tmp) / "out"
            into.mkdir()
            fetch.unpack_tar(archive, into)
            self.assertEqual((into / "lib" / "libmujoco.so").read_bytes(), b"x")
            self.assertFalse((into / "mujoco-9.9.9").exists())

    def test_zip_without_a_top_folder_stays(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / "a.zip"
            with zipfile.ZipFile(archive, "w") as z:
                z.writestr("lib/mujoco.lib", b"lib")
                # The real Windows release separates with backslashes.
                z.writestr("bin\\mujoco.dll", b"dll")
            into = Path(tmp) / "out"
            into.mkdir()
            fetch.unpack_zip(archive, into)
            self.assertEqual(sorted(p.name for p in into.iterdir()), ["bin", "lib"])
            self.assertEqual((into / "bin" / "mujoco.dll").read_bytes(), b"dll")

    def test_paths_that_escape_are_refused(self) -> None:
        for name in ["../evil", "/etc/passwd", "a/../../b", "C:/x", "..\\x"]:
            with self.assertRaises(SystemExit, msg=name):
                fetch.safe_member(name)
        self.assertEqual(fetch.safe_member("a/b"), PurePosixPath("a/b"))

    def test_single_top_folder(self) -> None:
        paths = [PurePosixPath(p) for p in ["m", "m/lib", "m/lib/x"]]
        self.assertEqual(fetch.single_top(paths), "m")
        self.assertIsNone(fetch.single_top([PurePosixPath("lib/x"), PurePosixPath("bin/y")]))


class PinTests(unittest.TestCase):
    def test_every_asset_is_pinned_and_matches_the_version(self) -> None:
        for (os_name, _), (asset, pin) in fetch.ASSETS.items():
            self.assertTrue(asset.startswith(f"mujoco-{fetch.VERSION}-"), asset)
            self.assertRegex(pin, r"^[0-9a-f]{64}$")
            self.assertIn(os_name, ("linux", "macos", "windows"))

    def test_version_matches_the_rust_binding(self) -> None:
        cargo = (fetch.REPO / "Cargo.toml").read_text()
        self.assertIn('mujoco-rs = { version = "=5.0.0"', cargo)
        config = (fetch.REPO / ".cargo" / "config.toml").read_text()
        self.assertIsNotNone(re.search(rf"mujoco-{re.escape(fetch.VERSION)}/lib", config))


if __name__ == "__main__":
    unittest.main()
