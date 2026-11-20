"""Import a trackgen build into Blender and export an FBX for ksEditor.

Command line (Blender 3.6+ / 4.x):
    blender --background --python blender/import_track.py -- output/<name>/source

or open Blender, paste this into the Text Editor, set SOURCE_DIR below and press Run.

Creates: <source>/track.blend and <source>/fbx/track.fbx (+ copied textures).
"""
import json
import math
import os
import shutil
import sys

import bpy

SOURCE_DIR = ""  # used when run from the Text Editor

# Assetto Corsa spawns a car facing the empty's local -Y axis (Blender frame,
# exported with forward=-Z / up=Y). If cars spawn backwards in game, set this to 180.
EXTRA_YAW_DEG = 0.0


def main():
    src = SOURCE_DIR
    if "--" in sys.argv:
        src = sys.argv[sys.argv.index("--") + 1]
    src = os.path.abspath(bpy.path.abspath(src))
    if not os.path.isfile(os.path.join(src, "track.obj")):
        raise SystemExit(f"track.obj not found in {src}")

    # start from an empty scene
    bpy.ops.wm.read_homefile(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"

    obj_path = os.path.join(src, "track.obj")
    if hasattr(bpy.ops.wm, "obj_import"):  # Blender 3.3+
        bpy.ops.wm.obj_import(filepath=obj_path, forward_axis="NEGATIVE_Z", up_axis="Y")
    else:
        bpy.ops.import_scene.obj(filepath=obj_path, axis_forward="-Z", axis_up="Y")

    # importer may append ".001" etc.; restore exact AC names
    for ob in list(scene.objects):
        base = ob.name.split(".")[0]
        if base != ob.name and base not in bpy.data.objects:
            ob.name = base

    markers = json.load(open(os.path.join(src, "markers.json")))["markers"]
    coll = bpy.data.collections.new("AC_markers")
    scene.collection.children.link(coll)
    for m in markers:
        e = bpy.data.objects.new(m["name"], None)
        e.empty_display_type = "ARROWS"
        e.empty_display_size = 1.5
        e.location = m["position"]
        fx, fy = m["forward"][0], m["forward"][1]
        # rotate about Z so that local -Y points along (fx, fy)
        e.rotation_euler = (0.0, 0.0, math.atan2(fx, -fy) + math.radians(EXTRA_YAW_DEG))
        coll.objects.link(e)

    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(src, "track.blend"))

    out_dir = os.path.join(src, "fbx")
    os.makedirs(out_dir, exist_ok=True)
    bpy.ops.export_scene.fbx(
        filepath=os.path.join(out_dir, "track.fbx"),
        object_types={"MESH", "EMPTY"},
        apply_scale_options="FBX_SCALE_ALL",
        axis_forward="-Z",
        axis_up="Y",
        use_mesh_modifiers=True,
        mesh_smooth_type="FACE",
        path_mode="COPY",
        embed_textures=False,
    )
    # ksEditor looks for textures next to the FBX
    tex = os.path.join(src, "textures")
    for f in os.listdir(tex):
        shutil.copy2(os.path.join(tex, f), out_dir)
    print(f"Exported {os.path.join(out_dir, 'track.fbx')}")


main()
