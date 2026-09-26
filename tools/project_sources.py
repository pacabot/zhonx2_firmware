"""Print the C sources referenced by the original Xcode project for Make."""

from pathlib import Path
import plistlib

ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = ROOT / "ZHONX_II_M4"
PROJECT = PROJECT_DIR / "ZHONX_II_M4.elf.xcodeproj/project.pbxproj"

with PROJECT.open("rb") as project_file:
    objects = plistlib.load(project_file)["objects"]

for entry in objects.values():
    if entry.get("isa") != "PBXFileReference":
        continue
    path = entry.get("path", "")
    if not path.endswith(".c"):
        continue
    source = (PROJECT_DIR / path).resolve().relative_to(ROOT)
    print(source.as_posix())
