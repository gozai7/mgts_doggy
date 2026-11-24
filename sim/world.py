import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import yaml

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
SCENES = sorted(path.stem for path in (CONFIGS / "scenes").glob("*.yaml")) + ["atrium-opaque"]


def layout_for(name, role):
    if name == "office":
        plan = yaml.safe_load((CONFIGS / "office.yaml").read_text())
        return {
            "bounds": plan["bounds"],
            "boxes": [*plan["walls"], plan["obstacle"][role]],
            "glass": [],
            "route": [],
        }
    if name not in SCENES:
        raise ValueError(f"unknown scene: {name}")
    plan = yaml.safe_load((CONFIGS / "scenes" / (name.removesuffix("-opaque") + ".yaml")).read_text())
    x0, y0, x1, y1 = plan["bounds"]
    perimeter = [[(x0 + x1) / 2, y, x1 - x0 + 0.16, 0.16] for y in (y0, y1)]
    perimeter += [[x, (y0 + y1) / 2, 0.16, y1 - y0] for x in (x0, x1)]
    panels = plan.get("glass", [])
    boxes = perimeter + plan["walls"] + panels
    return {
        "bounds": plan["bounds"],
        "boxes": boxes,
        "glass": [] if name.endswith("-opaque") else panels,
        "route": plan["routes"][role],
    }


def write_world(template, layout, destination):
    scene = ET.parse(template)
    world = scene.find("world")
    for model in world.findall("model"):
        if model.get("name") != "ground_plane":
            world.remove(model)
    for i, box in enumerate(layout["boxes"]):
        x, y, width, depth = box
        model = ET.SubElement(world, "model", name=f"wall_{i}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x} {y} 1 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        geometry = ET.Element("geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = f"{width} {depth} 2"
        for kind in ("collision", "visual"):
            part = ET.SubElement(link, kind, name=kind)
            part.append(deepcopy(geometry))
            if kind == "visual" and box in layout["glass"]:
                ET.SubElement(part, "transparency").text = "0.65"
    scene.write(destination)
