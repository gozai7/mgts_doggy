import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import yaml


def office(template, role, destination):
    layout = yaml.safe_load(Path("/mgts/configs/office.yaml").read_text())
    scene = ET.parse(template)
    world = scene.find("world")
    for model in world.findall("model"):
        if model.get("name") != "ground_plane":
            world.remove(model)
    boxes = [*layout["walls"], layout["obstacle"][role]]
    for i, (x, y, width, depth) in enumerate(boxes):
        model = ET.SubElement(world, "model", name=f"wall_{i}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x} {y} 1 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        geometry = ET.Element("geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = f"{width} {depth} 2"
        for kind in ("collision", "visual"):
            ET.SubElement(link, kind, name=kind).append(deepcopy(geometry))
    scene.write(destination)
    return dict(bounds=layout["bounds"], boxes=boxes)
