#!/usr/bin/env python3
"""Install signed upstream GLIM CUDA 12.6 binaries into a workspace-local prefix."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import urllib.request

REPOSITORY = "https://koide3.github.io/ppa/ubuntu2204/"
PACKAGES = {
    "libgtsam-notbb-dev": "4.3.0",
    "libgtsam-points-cuda12.6-dev": "1.2.2",
    "libiridescence-dev": "1.0.2",
    "ros-humble-glim-cuda12.6": "1.2.2-0jammy",
    "ros-humble-glim-ros-cuda12.6": "1.2.2-0jammy",
}


def run(*command):
    subprocess.run(command, check=True)


def install(workspace, proxy):
    if platform.machine() != "aarch64":
        raise RuntimeError("This deployment targets the Jetson aarch64 host.")
    workspace = workspace.expanduser().resolve(strict=True)
    root = workspace / "tools/glim_cuda"
    if root.exists() and root.is_symlink():
        raise RuntimeError("The deployment root must not be a symlink.")
    root.mkdir(parents=True, exist_ok=True)
    root = root.resolve()
    if not root.is_relative_to(workspace):
        raise RuntimeError("Deployment must stay inside the selected workspace.")
    cache = root / "packages"
    cache.mkdir(exist_ok=True)
    gpg_home = cache / "gnupg"
    gpg_home.mkdir(mode=0o700, exist_ok=True)
    manifest_file = root / "manifest.json"
    if manifest_file.exists():
        previous = json.loads(manifest_file.read_text())
        if {p["Package"]: p["Version"] for p in previous["packages"]} != PACKAGES:
            raise RuntimeError("Existing deployment has different versions; select another workspace.")
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({"http": proxy, "https": proxy})
        if proxy else urllib.request.ProxyHandler())

    def download(name, digest=None):
        destination = cache / Path(name).name
        if digest and destination.exists():
            if hashlib.sha256(destination.read_bytes()).hexdigest() == digest:
                return destination
        with opener.open(REPOSITORY + name.removeprefix("./"), timeout=60) as response:
            content = response.read()
        if digest and hashlib.sha256(content).hexdigest() != digest:
            raise RuntimeError("SHA256 mismatch: " + name)
        temporary = destination.with_suffix(destination.suffix + ".part")
        temporary.write_bytes(content)
        temporary.replace(destination)
        return destination

    key = download("KEY.gpg")
    release = download("Release")
    signature = download("Release.gpg")
    run("gpg", "--homedir", str(gpg_home), "--batch", "--yes", "--dearmor", "--output",
        str(cache / "upstream-keyring.gpg"), str(key))
    run("gpgv", "--homedir", str(gpg_home), "--keyring", str(cache / "upstream-keyring.gpg"),
        str(signature), str(release))
    hashes = re.search(r"(?m)^SHA256:\n((?: .+\n)+)", release.read_text())
    if hashes is None:
        raise RuntimeError("Signed release has no SHA256 index.")
    entries = [line.split() for line in hashes.group(1).splitlines()]
    digest = next(row[0] for row in entries if row[2] == "Packages.gz")
    index = download("Packages.gz", digest)
    records = []
    for block in re.split(r"\n(?=Package: )", gzip.decompress(index.read_bytes()).decode()):
        record = dict(re.findall(r"^([A-Za-z0-9-]+): (.*)$", block, re.M))
        if (record.get("Architecture") == "arm64" and
                PACKAGES.get(record.get("Package")) == record.get("Version")):
            records.append({k: record[k] for k in
                            ("Package", "Version", "Architecture", "Filename", "SHA256", "Depends")})
    if len(records) != len(PACKAGES):
        raise RuntimeError("Pinned packages are missing or duplicated in the signed index.")
    sysroot = root / "sysroot"
    sysroot.mkdir(exist_ok=True)
    for record in records:
        name = record["Filename"]
        if ".." in Path(name).parts or Path(name).is_absolute():
            raise RuntimeError("Unsafe archive filename in index.")
        archive = download(name, record["SHA256"])
        actual = subprocess.check_output(
            ["dpkg-deb", "-f", str(archive), "Package", "Version", "Architecture"], text=True)
        for field in ("Package", "Version", "Architecture"):
            if f'{field}: {record[field]}' not in actual:
                raise RuntimeError("Archive metadata mismatch: " + name)
        run("dpkg-deb", "-x", str(archive), str(sysroot))
        print(record["Package"], record["Version"], flush=True)
    # Iridescence needs GLFW even when the viewer extension is disabled.
    # Use the host's authenticated Ubuntu APT index for this small runtime only.
    runtime_records = []
    status = subprocess.run(["dpkg-query", "-W", "-f=${db:Status-Abbrev}", "libglfw3"],
                            text=True, capture_output=True)
    if status.returncode != 0 or not status.stdout.startswith("ii"):
        metadata = subprocess.check_output(
            ["apt-cache", "show", "--no-all-versions", "libglfw3"], text=True)
        record = dict(re.findall(r"^([A-Za-z0-9-]+): (.*)$", metadata.split("\n\n")[0], re.M))
        archive = cache / Path(record["Filename"]).name
        if not archive.exists() or hashlib.sha256(archive.read_bytes()).hexdigest() != record["SHA256"]:
            environment = dict(os.environ)
            if proxy:
                environment.update(http_proxy=proxy, https_proxy=proxy)
            subprocess.run(["apt-get", "download", "libglfw3=" + record["Version"]],
                           cwd=cache, env=environment, check=True)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != record["SHA256"]:
            raise RuntimeError("Ubuntu runtime SHA256 mismatch")
        run("dpkg-deb", "-x", str(archive), str(sysroot))
        runtime_records.append({k: record[k] for k in ("Package", "Version", "SHA256")})
    setup = root / "setup.bash"
    setup.write_text("""# Source this file; no system ROS or library installation is modified.
_glim_cuda_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export GLIM_CUDA_ROOT="$_glim_cuda_root"
export AMENT_PREFIX_PATH="$_glim_cuda_root/sysroot/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$_glim_cuda_root/sysroot/opt/ros/humble:$_glim_cuda_root/sysroot/usr:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_glim_cuda_root/sysroot/opt/ros/humble/lib:$_glim_cuda_root/sysroot/usr/lib:$_glim_cuda_root/sysroot/usr/lib/aarch64-linux-gnu:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PATH="$_glim_cuda_root/sysroot/opt/ros/humble/bin:$_glim_cuda_root/sysroot/usr/bin:$PATH"
unset _glim_cuda_root
""")
    manifest_file.write_text(json.dumps({
        "repository": REPOSITORY, "signature_verified": True,
        "release_sha256": hashlib.sha256(release.read_bytes()).hexdigest(),
        "packages": records, "runtime_packages": runtime_records,
    }, indent=2) + "\n")
    print("Deployment:", root, flush=True)
    print("Activate: source " + str(setup), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--proxy", default="")
    arguments = parser.parse_args()
    install(arguments.workspace, arguments.proxy)
