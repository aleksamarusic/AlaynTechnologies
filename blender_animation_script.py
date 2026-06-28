"""
Kinematic mirror mount + laser + target, with a self-aligning cap mechanism.

TIMELINE (tweakable constants below)
  Phase 1  frames 1..120     screws turn, mirror aligns, laser spot centres
  Phase 2  frames 135..165   a cap slides over the side screw
  Phase 3  frames 170..220   camera flies to face the round end of the cap
  Phase 4  frames 235..265   ~0.5 s settle, then cap fades + propeller revealed
  Phase 5  frames 270..SPIN  propeller spins a couple of turns; cap+screw react
  Phase 6  +60 frames        final hold (~2 s)

HOW TO USE
  Scripting workspace -> paste -> Run Script. Press Space to preview, then set
  RENDER = True. The spin phase length scales with SPIN_SPEED / PROP_TURN.
"""
import bpy
from math import radians, pi, cos, sin
from mathutils import Vector, Matrix

# ---- TWEAKABLES -------------------------------------------------------------
RENDER         = True
FPS            = 30
RES_X, RES_Y   = 1280, 720

ALIGN_FRAMES   = 120
CAP_START      = 135
CAP_END        = 165
CAM_HOLD_UNTIL = 170
CAM_END        = 220
FADE_START     = 235             # ~0.5 s after the camera settles
FADE_END       = 265
SPIN_START     = 270

SPIN_SPEED     = radians(4.5)   # rad/frame -- same feel as before (raise to shorten)
PROP_TURN      = 4 * pi          # a couple of turns
PROP_START_ANGLE = radians(35)   # offset so blades don't line up with the laser
REACT_TURN     = radians(120)    # cap+screw reaction (still well under PROP_TURN)
CAP_ALPHA      = 0.6
PROP_Z         = 0.39            # propeller depth: in front of knob (0.37), inside cap (0.41)

SPIN_FRAMES    = round(PROP_TURN / SPIN_SPEED)
SPIN_END       = SPIN_START + SPIN_FRAMES
HOLD2_END      = SPIN_END + 60   # ~2 s final hold
END_FRAME      = HOLD2_END

SCREW_A_TURNS  = 3.0
SCREW_B_TURNS  = -2.0
MISALIGN_X     = radians(5)
MISALIGN_Y     = radians(-3)
ROOT_TILT_X    = radians(-70)
ROOT_YAW_Z     = radians(15)
SIDE_SCREW_LOC = (1.10, 0.0, 0.55)

CAM_START_LOC  = (4.14321, 7.92511, 6.88676)
CAM_START_ROT  = (radians(52.4), radians(0), radians(152.4))
CAM_START_LENS = 55
FACE_DIST      = 2.6
FACE_LENS      = 80
# -----------------------------------------------------------------------------


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)


def mat(name, color, metallic=1.0, roughness=0.35):
    m = bpy.data.materials.new(name); m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*color, 1.0)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = roughness
    return m


def emission(name, color, strength):
    m = bpy.data.materials.new(name); m.use_nodes = True
    nt = m.node_tree; nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    em = nt.nodes.new('ShaderNodeEmission')
    em.inputs['Color'].default_value = (*color, 1.0)
    em.inputs['Strength'].default_value = strength
    nt.links.new(em.outputs['Emission'], out.inputs['Surface'])
    return m


def set_transparent(m):
    for prop, val in (("blend_method", 'BLEND'), ("surface_render_method", 'BLENDED')):
        if hasattr(m, prop):
            try:
                setattr(m, prop, val)
            except (TypeError, AttributeError):
                pass
    if hasattr(m, "show_transparent_back"):
        m.show_transparent_back = False


def assign(obj, material):
    obj.data.materials.clear(); obj.data.materials.append(material)


def cube(name, dims, location, material):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    o = bpy.context.active_object; o.name = name; o.scale = dims
    assign(o, material); return o


def cyl(name, radius, depth, location, material, verts=48):
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=depth,
                                        vertices=verts, location=location)
    o = bpy.context.active_object; o.name = name
    assign(o, material); return o


def torus(name, major, minor, location, material):
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor,
                                     location=location)
    o = bpy.context.active_object; o.name = name
    assign(o, material); return o


def sphere(name, radius, location, material):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=location)
    o = bpy.context.active_object; o.name = name
    assign(o, material); return o


def empty(name, location):
    e = bpy.data.objects.new(name, None); e.location = location
    bpy.context.collection.objects.link(e); return e


def parent_to(child, parent):
    bpy.context.view_layer.update()
    child.parent = parent
    child.matrix_parent_inverse = parent.matrix_world.inverted()


def parent_local(child, parent, loc=(0, 0, 0), rot=(0, 0, 0)):
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.location = loc
    child.rotation_euler = rot


def beam(name, a, b, radius, material):
    a, b = Vector(a), Vector(b); d = b - a
    o = cyl(name, radius, d.length, (a + b) / 2, material, verts=16)
    o.rotation_euler = d.to_track_quat('Z', 'Y').to_euler()
    return o


def make_front_knob(name, x, y, steel, anodized):
    cyl(f"{name}_shaft", 0.07, 0.45, (x, y, 0.55), steel, verts=20)
    knob = cyl(f"{name}_knob", 0.20, 0.34, (x, y, 0.74), steel, verts=40)
    for i in range(14):
        a = i / 14 * 2 * pi
        bpy.ops.mesh.primitive_cube_add(
            size=1, location=(x + 0.205 * cos(a), y + 0.205 * sin(a), 0.74))
        r = bpy.context.active_object; r.name = f"{name}_ridge_{i}"
        r.scale = (0.025, 0.04, 0.34); r.rotation_euler = (0, 0, a)
        assign(r, steel); parent_to(r, knob)
    bar = cube(f"{name}_mark", (0.34, 0.045, 0.02), (x, y, 0.92), anodized)
    parent_to(bar, knob)
    return knob


def make_side_knob(name, loc, steel, anodized):
    anchor = empty(f"{name}_anchor", loc)
    anchor.rotation_euler = (0, radians(90), 0)
    shaft = cyl(f"{name}_shaft", 0.07, 0.5, (0, 0, 0), steel, verts=20)
    parent_local(shaft, anchor, (0, 0, -0.10))
    knob = cyl(f"{name}_knob", 0.20, 0.34, (0, 0, 0), steel, verts=40)
    parent_local(knob, anchor, (0, 0, 0.20))
    for i in range(14):
        a = i / 14 * 2 * pi
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
        r = bpy.context.active_object; r.name = f"{name}_ridge_{i}"
        r.scale = (0.025, 0.04, 0.34); assign(r, steel)
        parent_local(r, knob, (0.205 * cos(a), 0.205 * sin(a), 0.0), (0, 0, a))
    bar = cube(f"{name}_mark", (0.34, 0.045, 0.02), (0, 0, 0), anodized)
    parent_local(bar, knob, (0, 0, 0.18))
    cap = cyl(f"{name}_cap", 0.24, 0.42, (0, 0, 0), anodized, verts=40)
    parent_local(cap, anchor, (0, 0, 0.70))
    return anchor, knob, cap


# ---- BUILD ------------------------------------------------------------------
clear_scene()
sc = bpy.context.scene
sc.render.fps = FPS; sc.frame_start = 1; sc.frame_end = END_FRAME

anodized = mat("Anodized", (0.012, 0.012, 0.014), 1.0, 0.45)
steel    = mat("Steel",    (0.62, 0.62, 0.64), 1.0, 0.28)
mirror   = mat("Mirror",   (0.80, 0.83, 0.88), 1.0, 0.02)
screen   = mat("Screen",   (0.06, 0.06, 0.07), 0.0, 0.8)
laser    = emission("Laser", (1.0, 0.05, 0.05), 6.0)
spot     = emission("Spot",  (1.0, 0.3, 0.3), 9.0)
ring_em  = emission("Ring",  (0.9, 0.2, 0.2), 1.5)
cap_mat  = mat("CapGlass", (0.02, 0.02, 0.025), 0.6, 0.25)
prop_mat = emission("Propeller", (0.10, 0.30, 1.00), 4.0)    # blue, self-lit

root  = empty("MountRoot", (0, 0, 0))
pivot = empty("TiltPivot", (-0.95, 0.95, 0.5))

back  = cube("BackPlate",  (2.2, 2.2, 0.18), (0, 0, 0.0), anodized)
front = cube("FrontPlate", (2.2, 2.2, 0.18), (0, 0, 0.5), anodized)
disc  = cyl("MirrorDisc", 0.70, 0.08, (0, 0, 0.60), mirror, verts=64)
ring  = torus("RetainRing", 0.74, 0.05, (0, 0, 0.59), anodized)
ball  = sphere("PivotBall", 0.09, (-0.95, 0.95, 0.30), steel)

knobA = make_front_knob("ScrewA", 0.0, -1.00, steel, anodized)
anchorB, knobB, capB = make_side_knob("ScrewB", SIDE_SCREW_LOC, steel, anodized)
assign(capB, cap_mat); set_transparent(cap_mat)
cap_bsdf = cap_mat.node_tree.nodes.get("Principled BSDF")

# propeller: a hub + two pitched blades, on the screw axis, spins independently
prop = empty("Propeller", (0, 0, 0))
parent_local(prop, anchorB, (0, 0, PROP_Z))
hub = cyl("PropHub", 0.05, 0.035, (0, 0, 0), prop_mat, verts=16)
parent_local(hub, prop, (0, 0, 0))
bladeA = cube("PropBladeA", (0.18, 0.05, 0.012), (0, 0, 0), prop_mat)
parent_local(bladeA, prop, (0.13, 0, 0), (radians(25), 0, 0))
bladeB = cube("PropBladeB", (0.18, 0.05, 0.012), (0, 0, 0), prop_mat)
parent_local(bladeB, prop, (-0.13, 0, 0), (radians(25), 0, 0))
prop_parts = (hub, bladeA, bladeB)

src_pos = (-3.2, -0.6, 2.0)
M = (0.0, 0.0, 0.62)
cube("LaserHead", (0.35, 0.35, 0.7), src_pos, anodized)
beam_in  = beam("BeamIn", src_pos, M, 0.035, laser)
beam_out = beam("BeamOut", M, (0.0, 0.0, 3.9), 0.035, laser)
tip = sphere("LaserSpot", 0.07, (0.0, 0.0, 3.9), spot)

bpy.ops.mesh.primitive_plane_add(size=1.7, location=(0, 0, 4.0))
target = bpy.context.active_object; target.name = "TargetScreen"; assign(target, screen)
torus("Bull1", 0.20, 0.018, (0, 0, 3.97), ring_em)
torus("Bull2", 0.40, 0.018, (0, 0, 3.97), ring_em)

# ---- HIERARCHY --------------------------------------------------------------
for o in (front, disc, ring):
    parent_to(o, pivot)
for o in (beam_out, tip):
    parent_to(o, pivot)
for o in (pivot, back, ball, knobA, anchorB, target):
    parent_to(o, root)
parent_to(bpy.data.objects["LaserHead"], root)
parent_to(beam_in, root)
for n in ("Bull1", "Bull2"):
    parent_to(bpy.data.objects[n], root)

root.rotation_euler = (ROOT_TILT_X, 0, ROOT_YAW_Z)
bpy.context.view_layer.update()
screw_world = knobB.matrix_world.translation.copy()
screw_axis  = knobB.matrix_world.to_3x3().col[2].normalized()

# ---- ANIMATION --------------------------------------------------------------
def spin(knob, turns):
    knob.rotation_euler = (0, 0, 0)
    knob.keyframe_insert("rotation_euler", frame=1)
    knob.rotation_euler = (0, 0, turns * 2 * pi)
    knob.keyframe_insert("rotation_euler", frame=ALIGN_FRAMES)

spin(knobA, SCREW_A_TURNS)
spin(knobB, SCREW_B_TURNS)

pivot.rotation_euler = (MISALIGN_X, MISALIGN_Y, 0)
pivot.keyframe_insert("rotation_euler", frame=1)
pivot.rotation_euler = (0, 0, 0)
pivot.keyframe_insert("rotation_euler", frame=ALIGN_FRAMES)

# cap slides on
capB.hide_viewport = capB.hide_render = True
capB.keyframe_insert("hide_viewport", frame=1)
capB.keyframe_insert("hide_render", frame=1)
capB.hide_viewport = capB.hide_render = False
capB.keyframe_insert("hide_viewport", frame=CAP_START)
capB.keyframe_insert("hide_render", frame=CAP_START)
capB.location = (0, 0, 0.70); capB.keyframe_insert("location", frame=CAP_START)
capB.location = (0, 0, 0.20); capB.keyframe_insert("location", frame=CAP_END)

# cap fades to semitransparent
cap_bsdf.inputs["Alpha"].default_value = 1.0
cap_bsdf.inputs["Alpha"].keyframe_insert("default_value", frame=FADE_START)
cap_bsdf.inputs["Alpha"].default_value = CAP_ALPHA
cap_bsdf.inputs["Alpha"].keyframe_insert("default_value", frame=FADE_END)

# propeller: parts hidden until reveal, then the assembly spins a couple of turns
for p in prop_parts:
    p.hide_viewport = p.hide_render = True
    p.keyframe_insert("hide_viewport", frame=1)
    p.keyframe_insert("hide_render", frame=1)
    p.hide_viewport = p.hide_render = False
    p.keyframe_insert("hide_viewport", frame=FADE_START)
    p.keyframe_insert("hide_render", frame=FADE_START)
prop.rotation_euler = (0, 0, PROP_START_ANGLE)
prop.keyframe_insert("rotation_euler", frame=SPIN_START)
prop.rotation_euler = (0, 0, PROP_START_ANGLE + PROP_TURN)
prop.keyframe_insert("rotation_euler", frame=SPIN_END)

# reaction: cap + screw rotate together, opposite and smaller than the propeller
capB.rotation_euler = (0, 0, 0); capB.keyframe_insert("rotation_euler", frame=SPIN_START)
capB.rotation_euler = (0, 0, -REACT_TURN); capB.keyframe_insert("rotation_euler", frame=SPIN_END)
base_z = SCREW_B_TURNS * 2 * pi
knobB.rotation_euler = (0, 0, base_z); knobB.keyframe_insert("rotation_euler", frame=SPIN_START)
knobB.rotation_euler = (0, 0, base_z - REACT_TURN); knobB.keyframe_insert("rotation_euler", frame=SPIN_END)

# ---- CAMERA (animated) ------------------------------------------------------
target_e = empty("CamTarget", (0, 0, 0)); parent_to(target_e, root)
target_e.location = (0, 0, 2.0)

cam_data = bpy.data.cameras.new("Cam"); cam_data.lens = CAM_START_LENS
cam = bpy.data.objects.new("Camera", cam_data)
bpy.context.collection.objects.link(cam); sc.camera = cam

cam.location = CAM_START_LOC; cam.rotation_euler = CAM_START_ROT
cam.keyframe_insert("location", frame=1)
cam.keyframe_insert("rotation_euler", frame=1)
cam.keyframe_insert("location", frame=CAM_HOLD_UNTIL)
cam.keyframe_insert("rotation_euler", frame=CAM_HOLD_UNTIL)
cam_data.keyframe_insert("lens", frame=1)
cam_data.keyframe_insert("lens", frame=CAM_HOLD_UNTIL)

cam_final = screw_world + screw_axis * FACE_DIST
cam.location = cam_final
cam.rotation_euler = screw_axis.to_track_quat('Z', 'Y').to_euler()
cam.keyframe_insert("location", frame=CAM_END)
cam.keyframe_insert("rotation_euler", frame=CAM_END)
cam_data.lens = FACE_LENS
cam_data.keyframe_insert("lens", frame=CAM_END)
# last camera keyframe is at CAM_END; constant extrapolation holds it for every later phase

# ---- LIGHTS (derived from the main-shot camera) -----------------------------
cam_v = Vector(CAM_START_LOC)

def area_light(name, location, energy, size=6):
    ld = bpy.data.lights.new(name, 'AREA'); ld.energy = energy; ld.size = size
    lo = bpy.data.objects.new(name, ld); lo.location = location
    bpy.context.collection.objects.link(lo)
    lo.constraints.new(type='TRACK_TO').target = target_e
    return lo

area_light("Key",  cam_v + Vector((-1.5, -1.0, 3.0)), 2200, size=7)
area_light("Fill", cam_v * 0.2 + Vector((-5, -4, 1)), 500, size=10)
area_light("Rim",  -cam_v * 0.5 + Vector((0, 0, 5)), 1300, size=4)

cap_target = empty("CapTarget", screw_world)
ld = bpy.data.lights.new("FaceLight", 'AREA'); ld.energy = 1200; ld.size = 2.5
fl = bpy.data.objects.new("FaceLight", ld)
fl.location = screw_world + screw_axis * (FACE_DIST * 0.7) + Vector((0, 0, 2.2))
bpy.context.collection.objects.link(fl)
fl.constraints.new(type='TRACK_TO').target = cap_target

w = sc.world; w.use_nodes = True
w.node_tree.nodes["Background"].inputs[0].default_value = (0.03, 0.03, 0.04, 1)

# ---- RENDER SETTINGS --------------------------------------------------------
for eng in ('BLENDER_EEVEE_NEXT', 'BLENDER_EEVEE', 'CYCLES'):
    try:
        sc.render.engine = eng; break
    except TypeError:
        continue
for attr in ('use_ssr', 'use_raytracing', 'use_bloom', 'use_gtao'):
    if hasattr(sc.eevee, attr):
        setattr(sc.eevee, attr, True)
if hasattr(sc.eevee, 'use_ssr_refraction'):
    sc.eevee.use_ssr_refraction = True
if hasattr(sc.eevee, 'taa_render_samples'):
    sc.eevee.taa_render_samples = 128

sc.render.resolution_x = RES_X; sc.render.resolution_y = RES_Y

if RENDER:
    sc.render.image_settings.file_format = 'PNG'
    sc.render.filepath = "//frames/frame_"
    bpy.ops.render.render(animation=True)
    print(f"Rendered {END_FRAME} PNG frames to //frames/")
else:
    print(f"Scene built ({END_FRAME} frames). Press Space to preview.")
