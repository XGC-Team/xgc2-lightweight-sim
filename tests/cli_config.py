"""Finite native CLI/config and Unix startup checks; no ROS graph or experiment."""
import argparse
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import tempfile
import time

from native_http import UnixHTTP


def main(binary):
    with tempfile.TemporaryDirectory(prefix="xsim-cli-config-") as temporary:
        root = Path(temporary)
        config = root / "world.json"
        scene = root / "scene.yaml"
        scene.write_text("obstacles: []\n")
        original = {"instance_id": "cli-config-test", "epoch_ns": 1791250000000000000,
                    "paused": True, "entities": []}
        config.write_text(json.dumps(original))
        socket_path = root / "new" / "nested" / "world.sock"
        command = [binary, "--config", str(config), "--socket", str(socket_path)]

        def rejected(arguments, reason):
            result = subprocess.run(arguments, capture_output=True, text=True, timeout=5)
            assert result.returncode == 1, (arguments, result)
            assert reason in result.stderr, result.stderr
            assert not socket_path.exists(), "rejected config bound the socket"

        assert "--scene-file" in subprocess.check_output([binary, "--help"], text=True)
        rejected(command + ["--scene-file"], "missing value")
        rejected(command + ["--scene-file", "", "--scene-file", ""], "more than once")
        rejected(command + ["--config", str(config)], "more than once")
        rejected(command + ["--socket", str(socket_path)], "more than once")
        rejected([binary, "--config", "--socket", str(socket_path)], "missing value")
        rejected([binary, "--config", str(config), "--socket"], "missing value")
        rejected(command + ["--scene-file", str(root / "absent.yaml")], "bad file")
        configured = dict(original, scene_file=str(scene))
        config.write_text(json.dumps(configured))
        rejected(command + ["--scene-file", str(root / "different.yaml")], "conflicts")
        scene.write_text("obstacles: [unterminated\n")
        rejected(command + ["--scene-file", str(scene)], "end of")
        for epoch in [0, -1, 1.5, 18446744073709551615, 9223372036854775807]:
            config.write_text(json.dumps(dict(original, epoch_ns=epoch)))
            rejected(command, "epoch")
        missing_epoch = dict(original)
        del missing_epoch["epoch_ns"]
        config.write_text(json.dumps(missing_epoch))
        rejected(command, "epoch_ns")
        for text in ['{} {}', '{"epoch_ns":1,"epoch_ns":2}', '{"x":{"a":1,"a":2}}']:
            config.write_text(text)
            rejected(command, "configuration" if 'epoch_ns' in text or '"x"' in text else "parse")
        config.write_text(json.dumps(original))

        def start(arguments):
            process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not socket_path.exists() and process.poll() is None:
                time.sleep(.01)
            assert process.poll() is None and socket_path.exists(), process.communicate(timeout=1)
            connection = UnixHTTP(str(socket_path))
            try:
                connection.request("GET", "/config")
                response = connection.getresponse()
                assert response.status == 200
                value = json.loads(response.read())
                assert value["world"]["epoch_ns"] == original["epoch_ns"], value
            finally:
                connection.close()
            return process

        # The common catalog always supplies its optional scene argument, even empty.
        process = start(command + ["--scene-file", ""])
        try:
            assert stat.S_IMODE(socket_path.parent.stat().st_mode) == 0o700
            assert stat.S_IMODE(socket_path.parent.parent.stat().st_mode) == 0o700
            assert stat.S_IMODE(socket_path.stat().st_mode) == 0o600
        finally:
            process.terminate()
            _, stderr = process.communicate(timeout=5)
            assert process.returncode == 0, stderr
        assert not socket_path.exists(), "normal shutdown leaked owned socket"
        assert socket_path.parent.exists(), "shutdown removed its parent directory"

        scene.write_text("obstacles: []\n")
        for arguments, value in [(command + ["--scene-file", str(scene)], original),
                                 (command, dict(original, scene_file=str(scene)))]:
            config.write_text(json.dumps(value))
            process = start(arguments)
            process.terminate()
            _, stderr = process.communicate(timeout=5)
            assert process.returncode == 0, stderr
            assert not socket_path.exists()
        config.write_text(json.dumps(original))

        # Pre-existing foreign sockets/files must not be removed even on a failed bind.
        with socket.socket(socket.AF_UNIX) as foreign:
            foreign.bind(str(socket_path))
            identity = socket_path.lstat().st_ino
            result = subprocess.run(command, capture_output=True, timeout=5)
            assert result.returncode == 1 and b"path must be absent" in result.stderr
            assert socket_path.lstat().st_ino == identity
        socket_path.unlink()
        socket_path.write_text("foreign-file")
        result = subprocess.run(command, capture_output=True, timeout=5)
        assert result.returncode == 1 and socket_path.read_text() == "foreign-file"
        socket_path.unlink()

        # A pathname replacement after bind remains foreign at our RAII cleanup.
        process = start(command)
        moved = socket_path.with_name("moved-original.sock")
        try:
            socket_path.rename(moved)
            with socket.socket(socket.AF_UNIX) as replacement:
                replacement.bind(str(socket_path))
                identity = socket_path.lstat().st_ino
                process.terminate()
                _, stderr = process.communicate(timeout=5)
                assert process.returncode == 0, stderr
                assert socket_path.lstat().st_ino == identity
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=5)
        print(json.dumps({"ok": True, "emptySceneFlag": True, "validSceneFlagAndConfig": True,
                          "sceneConflictRefused": True,
                          "invalidSceneUsesOriginalParser": True, "frozenEpochUnchanged": True,
                          "privateParents": True, "foreignSocketAndReplacementPreserved": True}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--xsim", required=True)
    main(os.path.abspath(parser.parse_args().xsim))
