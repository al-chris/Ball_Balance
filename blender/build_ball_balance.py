"""
Ball-on-Plate balancing rig -- parametric Blender build script.

Mechanism (2-DOF):
  * Ground base with a centre post topped by a universal joint (Cardan).
  * Square tilting plate (aluminium + resistive-touch glass + rim) on the U-joint.
  * Two hobby servos (X and Y). Each servo horn drives a push rod with spherical
    rod ends up to a clevis under the plate.
  * 40 mm chrome steel ball.

Every rigid body lives in its own collection and is parented to a frame Empty
placed on its joint, so the model can be:
  * posed/animated in Blender by rotating the frame Empties, and
  * exported per body (STL, metres, body-local frame) for Simscape Multibody.

Run (from the project root):
  blender --background --factory-startup --python blender/build_ball_balance.py -- . [--render]

World axes: Z up, base top surface at z = 0, units = metres.
"""
import bpy, bmesh, math, os, sys, struct
from mathutils import Vector, Matrix

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
pos_args = [a for a in argv if not a.startswith("--")]
ROOT = os.path.abspath(pos_args[0]) if pos_args else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DO_RENDER = "--render" in argv
PREVIEW = "--preview" in argv

# ---------------------------------------------------------------------------
# Key dimensions (m) -- the same numbers are written to cad/bb_geometry.m
# ---------------------------------------------------------------------------
PIVOT_Z = 0.160          # U-joint centre height above base top
SERVO_R = 0.075          # servo shaft offset from centre (along X / Y)
SERVO_Z = 0.060          # servo shaft height
HORN_L = 0.025           # horn crank radius
ATTACH_R = SERVO_R + HORN_L   # plate attachment radius (0.100) -> vertical rod
ATTACH_Z = 0.150         # plate clevis pin height (neutral)
ROD_L = ATTACH_Z - SERVO_Z    # rod length between joint centres (0.090)
PLATE_BOT = 0.178
PLATE_T = 0.006
GLASS_TOP = PLATE_BOT + PLATE_T + 0.002   # 0.186, rolling surface
PLATE_W = 0.250
GLASS_W = 0.225
BALL_R = 0.020
BALL_XY = (0.035, -0.040)

# ---------------------------------------------------------------------------
# Scene reset
# ---------------------------------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene.unit_settings.length_unit = "MILLIMETERS"

# ---------------------------------------------------------------------------
# Geometry helpers (bmesh, built directly in world coordinates)
# ---------------------------------------------------------------------------
def T(x=0.0, y=0.0, z=0.0):
    return Matrix.Translation((x, y, z))

def R(axis, deg):
    return Matrix.Rotation(math.radians(deg), 4, axis)

# profile-plane mappings for extruded 2-D profiles: local (u, v, w) -> world
M_XY_Z = Matrix.Identity(4)
M_XZ_Y = Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
M_YZ_X = Matrix(((0, 0, 1, 0), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
ALONG_X = R("Y", 90)
ALONG_Y = R("X", -90)


def finish(bm):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


def xf(bm, M):
    bmesh.ops.transform(bm, matrix=M, verts=bm.verts[:])
    return bm


def bevel_sharp(bm, offset, segments=3, angle=35):
    if offset <= 0:
        return bm
    bm.normal_update()
    edges = [e for e in bm.edges if e.is_manifold and e.calc_face_angle(0) > math.radians(angle)]
    if edges:
        bmesh.ops.bevel(bm, geom=edges, offset=offset, segments=segments,
                        affect="EDGES", profile=0.5, clamp_overlap=True)
    return bm


def merge(*bms):
    out = bmesh.new()
    for b in bms:
        vm = {v: out.verts.new(v.co) for v in b.verts}
        for f in b.faces:
            out.faces.new([vm[v] for v in f.verts])
        b.free()
    return finish(out)


def box(x0, x1, y0, y1, z0, z1, bevel=0.0, corner=0.0, cseg=8, seg=3):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co.x = x0 if v.co.x < 0 else x1
        v.co.y = y0 if v.co.y < 0 else y1
        v.co.z = z0 if v.co.z < 0 else z1
    if corner > 0:
        ve = [e for e in bm.edges if abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-9]
        bmesh.ops.bevel(bm, geom=ve, offset=corner, segments=cseg, affect="EDGES",
                        profile=0.5, clamp_overlap=True)
    bevel_sharp(bm, bevel, seg)
    return finish(bm)


def cyl(r, z0, z1, seg=48, bevel=0.0, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg,
                          radius1=r, radius2=r if r2 is None else r2, depth=z1 - z0)
    bmesh.ops.translate(bm, vec=(0, 0, (z0 + z1) / 2), verts=bm.verts[:])
    bevel_sharp(bm, bevel, 2, angle=40)
    return finish(bm)


def sphere(r, u=64, v=32):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=u, v_segments=v, radius=r)
    return finish(bm)


def prism(pts, w0, w1, bevel=0.0):
    bm = bmesh.new()
    b = [bm.verts.new((u, v, w0)) for u, v in pts]
    t = [bm.verts.new((u, v, w1)) for u, v in pts]
    bm.faces.new(b[::-1])
    bm.faces.new(t)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((b[i], b[j], t[j], t[i]))
    finish(bm)
    bevel_sharp(bm, bevel, 2)
    return finish(bm)


def ring(outer, inner, w0, w1, bevel=0.0):
    """Extruded ring between two closed 2-D loops with equal point counts."""
    bm = bmesh.new()
    loops = [[bm.verts.new((u, v, w)) for u, v in pts]
             for pts, w in ((outer, w0), (outer, w1), (inner, w1), (inner, w0))]
    n = len(outer)
    for k in range(4):
        a, b = loops[k], loops[(k + 1) % 4]
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((a[i], a[j], b[j], b[i]))
    finish(bm)
    bevel_sharp(bm, bevel, 2)
    return finish(bm)


def circle_pts(r, n=48):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def tube(r_in, r_out, z0, z1, n=48):
    return ring(circle_pts(r_out, n), circle_pts(r_in, n), z0, z1)


def arc(cx, cy, r, a0, a1, n):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


def rrect(w, h, r, n=8):
    hw, hh = w / 2, h / 2
    return (arc(hw - r, hh - r, r, 0, math.pi / 2, n) + arc(-hw + r, hh - r, r, math.pi / 2, math.pi, n)
            + arc(-hw + r, -hh + r, r, math.pi, 1.5 * math.pi, n)
            + arc(hw - r, -hh + r, r, 1.5 * math.pi, 2 * math.pi, n))


def rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def ear(hw, v_base, v_c, u_c=0.0, n=16):
    """Yoke/clevis ear: rectangle from v_base to a semicircle centred at v_c."""
    if v_base < v_c:   # rounded top
        return [(u_c - hw, v_base), (u_c + hw, v_base)] + arc(u_c, v_c, hw, 0, math.pi, n)
    return [(u_c + hw, v_base), (u_c - hw, v_base)] + arc(u_c, v_c, hw, math.pi, 2 * math.pi, n)


def slot(r1, r2, length, n=16):
    return arc(length, 0, r2, -math.pi / 2, math.pi / 2, n) + arc(0, 0, r1, math.pi / 2, 1.5 * math.pi, n)


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def material(name, color, metallic=0.0, rough=0.5, density=2700.0, aniso=0.0,
             coat=0.0, emission=None, rough_noise=None):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except Exception:
        pass
    m.diffuse_color = (*color, 1.0)
    m.metallic = metallic
    m.roughness = rough
    m["density"] = density
    nt = m.node_tree
    bsdf = nt.nodes.get("Principled BSDF") if nt else None
    if bsdf is None:
        return m

    def s(key, val):
        if key in bsdf.inputs:
            bsdf.inputs[key].default_value = val

    s("Base Color", (*color, 1.0))
    s("Metallic", metallic)
    s("Roughness", rough)
    if aniso:
        s("Anisotropic", aniso)
    if coat:
        s("Coat Weight", coat)
        s("Coat Roughness", 0.03)
    if emission:
        s("Emission Color", (*emission, 1.0))
        s("Emission Strength", 6.0)
    if rough_noise:   # micro roughness variation (machined / brushed look)
        lo, hi, scale = rough_noise
        tc = nt.nodes.new("ShaderNodeTexCoord")
        mp = nt.nodes.new("ShaderNodeMapping")
        mp.inputs["Scale"].default_value = (1.0, 1.0, 40.0)
        nz = nt.nodes.new("ShaderNodeTexNoise")
        nz.inputs["Scale"].default_value = scale
        nz.inputs["Detail"].default_value = 8.0
        cr = nt.nodes.new("ShaderNodeValToRGB")
        cr.color_ramp.elements[0].color = (lo, lo, lo, 1)
        cr.color_ramp.elements[1].color = (hi, hi, hi, 1)
        nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
        nt.links.new(mp.outputs["Vector"], nz.inputs["Vector"])
        nt.links.new(nz.outputs["Fac"], cr.inputs["Fac"])
        nt.links.new(cr.outputs["Color"], bsdf.inputs["Roughness"])
    return m


MAT = {
    "alu":       material("Aluminium_Brushed", (0.80, 0.81, 0.83), 1.0, 0.28, 2700, aniso=0.5, rough_noise=(0.20, 0.36, 180)),
    "alu_dark":  material("Aluminium_Anodized_Graphite", (0.055, 0.060, 0.068), 1.0, 0.38, 2700, rough_noise=(0.32, 0.45, 220)),
    "alu_black": material("Aluminium_Anodized_Black", (0.018, 0.018, 0.020), 1.0, 0.33, 2700),
    "alu_blue":  material("Aluminium_Anodized_Blue", (0.03, 0.16, 0.62), 1.0, 0.30, 2700),
    "steel":     material("Steel_Stainless", (0.70, 0.70, 0.72), 1.0, 0.18, 7900),
    "chrome":    material("Chrome", (0.92, 0.92, 0.93), 1.0, 0.035, 7850),
    "brass":     material("Brass", (0.85, 0.62, 0.32), 1.0, 0.25, 8500),
    "servo":     material("Servo_Plastic", (0.018, 0.018, 0.020), 0.0, 0.42, 1800),
    "label":     material("Servo_Label", (0.05, 0.22, 0.75), 0.0, 0.35, 1200),
    "glass":     material("Touch_Glass", (0.010, 0.012, 0.016), 0.0, 0.04, 2500, coat=1.0),
    "rubber":    material("Rubber", (0.012, 0.012, 0.012), 0.0, 0.85, 1100),
    "pcb":       material("PCB_Teal", (0.0, 0.33, 0.36), 0.0, 0.45, 1850),
    "chip":      material("IC_Black", (0.015, 0.015, 0.017), 0.0, 0.55, 1800),
    "cable":     material("Cable_Jacket", (0.025, 0.022, 0.020), 0.0, 0.6, 1300),
    "led":       material("LED_Green", (0.1, 1.0, 0.25), 0.0, 0.3, 1200, emission=(0.1, 1.0, 0.25)),
    "print":     material("Print_White", (0.75, 0.76, 0.78), 0.0, 0.5, 1200),
    "backdrop":  material("Studio_Backdrop", (0.16, 0.17, 0.19), 0.0, 0.7, 0),
}

# ---------------------------------------------------------------------------
# Collections, frames, objects
# ---------------------------------------------------------------------------
def collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or scene.collection).children.link(c)
    return c


C_RIG = collection("BallBalance_Rig")
C_FRAMES = collection("Frames", C_RIG)
C_ENV = collection("Studio")
BODY = {}      # body name -> (collection, frame empty)
FRAME_W = {}   # empty name -> world position (all frames are translation-only at neutral)


def frame(name, pos, parent=None, size=0.02):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = "ARROWS"
    e.empty_display_size = size
    C_FRAMES.objects.link(e)
    pos = Vector(pos)
    FRAME_W[name] = pos
    if parent is not None:
        e.parent = parent
        e.location = pos - FRAME_W[parent.name]
    else:
        e.location = pos
    return e


def body(name, frame_obj):
    c = collection(name, C_RIG)
    BODY[name] = (c, frame_obj)
    return c


def add(body_name, name, bm, mat, smooth=True, angle=35):
    coll, fr = BODY[body_name]
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.transform(T(*(-FRAME_W[fr.name])))
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    ob.parent = fr
    me.materials.append(mat)
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
        try:
            me.set_sharp_from_angle(angle=math.radians(angle))
        except Exception:
            pass
    return ob


def cable(body_name, name, pts, mat, r=0.0011):
    coll, fr = BODY[body_name]
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = r
    cu.bevel_resolution = 4
    cu.resolution_u = 24
    cu.use_fill_caps = True
    sp = cu.splines.new("BEZIER")
    sp.bezier_points.add(len(pts) - 1)
    off = FRAME_W[fr.name]
    for bp, p in zip(sp.bezier_points, pts):
        bp.co = Vector(p) - off
        bp.handle_left_type = bp.handle_right_type = "AUTO"
    ob = bpy.data.objects.new(name, cu)
    ob.data.materials.append(mat)
    coll.objects.link(ob)
    ob.parent = fr
    ob["no_export"] = True
    return ob


# Frames: kinematic tree Base -> Cross -> Plate, Base -> Horn_X/Y.  Rods follow
# horn tip and plate clevis through constraints, so posing horns + plate in
# Blender keeps the linkage visually closed.
F_BASE = frame("F_Base", (0, 0, 0), size=0.05)
F_CROSS = frame("F_Cross", (0, 0, PIVOT_Z), F_BASE)
F_PLATE = frame("F_Plate", (0, 0, PIVOT_Z), F_CROSS, size=0.04)
F_BALL = frame("F_Ball", (*BALL_XY, GLASS_TOP + BALL_R), size=0.03)

body("Base", F_BASE)
body("UJoint_Cross", F_CROSS)
body("Plate", F_PLATE)
body("Ball", F_BALL)

# ---------------------------------------------------------------------------
# Base (ground)
# ---------------------------------------------------------------------------
add("Base", "Base_Plate", box(-0.16, 0.16, -0.16, 0.16, -0.012, 0.0, bevel=0.0015, corner=0.018), MAT["alu_dark"])
feet = [cyl(0.013, -0.022, -0.012, bevel=0.002) for _ in range(4)]
for bm, (sx, sy) in zip(feet, [(1, 1), (-1, 1), (-1, -1), (1, -1)]):
    xf(bm, T(0.13 * sx, 0.13 * sy, 0))
add("Base", "Base_Feet", merge(*feet), MAT["rubber"])
screws = []
for sx, sy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
    screws.append(xf(cyl(0.0038, 0.0, 0.0012, bevel=0.0004), T(0.145 * sx, 0.145 * sy, 0)))
    screws.append(xf(cyl(0.0016, 0.0011, 0.0013, seg=6), T(0.145 * sx, 0.145 * sy, 0)))
add("Base", "Base_CornerScrews", merge(*screws), MAT["steel"])

# centre post + lower yoke
add("Base", "Post_Flange", cyl(0.024, 0.0, 0.006, bevel=0.001), MAT["alu"])
bolts = [xf(cyl(0.0028, 0.006, 0.0085, bevel=0.0004), T(0.0175 * math.cos(a), 0.0175 * math.sin(a), 0))
         for a in [math.pi / 4 + k * math.pi / 2 for k in range(4)]]
add("Base", "Post_FlangeBolts", merge(*bolts), MAT["steel"])
add("Base", "Post_Shaft", cyl(0.008, 0.006, 0.136), MAT["chrome"])
add("Base", "Post_Collar", merge(cyl(0.011, 0.130, 0.142, bevel=0.0008),
                                 xf(xf(cyl(0.0018, 0.0, 0.0025, seg=6), ALONG_X), T(0.0105, 0, 0.136))),
    MAT["alu"])
add("Base", "UJoint_LowerYoke", merge(
    box(-0.016, 0.016, -0.0095, 0.0095, 0.142, 0.149, bevel=0.0008),
    xf(prism(ear(0.007, 0.1485, PIVOT_Z), 0.0095, 0.0145, bevel=0.0004), M_YZ_X),
    xf(prism(ear(0.007, 0.1485, PIVOT_Z), -0.0145, -0.0095, bevel=0.0004), M_YZ_X)), MAT["alu"])

# ---------------------------------------------------------------------------
# Universal-joint cross (spider) -- rotates about world X relative to Base
# ---------------------------------------------------------------------------
P = T(0, 0, PIVOT_Z)
add("UJoint_Cross", "Cross_Spider", merge(
    xf(box(-0.0045, 0.0045, -0.0045, 0.0045, -0.0045, 0.0045, bevel=0.0008), P),
    xf(xf(cyl(0.0024, -0.0158, 0.0158), ALONG_X), P),
    xf(xf(cyl(0.0024, -0.0158, 0.0158), ALONG_Y), P)), MAT["steel"])
caps = []
for s in (1, -1):
    caps.append(xf(xf(cyl(0.0040, 0.0, 0.0013, bevel=0.0003), ALONG_X), P @ T(0.0145 * s if s > 0 else -0.0158, 0, 0)))
    caps.append(xf(xf(cyl(0.0040, 0.0, 0.0013, bevel=0.0003), ALONG_Y), P @ T(0, 0.0145 if s > 0 else -0.0158, 0)))
add("UJoint_Cross", "Cross_BearingCaps", merge(*caps), MAT["alu_black"])

# ---------------------------------------------------------------------------
# Plate assembly -- rotates about world Y relative to the cross
# ---------------------------------------------------------------------------
add("Plate", "Plate_Hub", merge(
    cyl(0.022, 0.1752, PLATE_BOT, bevel=0.0006),
    box(-0.009, 0.009, -0.016, 0.016, 0.1705, 0.1755, bevel=0.0006),
    xf(prism(ear(0.007, 0.171, PIVOT_Z), 0.0095, 0.0145, bevel=0.0004), M_XZ_Y),
    xf(prism(ear(0.007, 0.171, PIVOT_Z), -0.0145, -0.0095, bevel=0.0004), M_XZ_Y)), MAT["alu"])
add("Plate", "Plate_Deck", box(-PLATE_W / 2, PLATE_W / 2, -PLATE_W / 2, PLATE_W / 2, PLATE_BOT, PLATE_BOT + PLATE_T,
                               bevel=0.0008, corner=0.012), MAT["alu"])
add("Plate", "Plate_Rim", ring(rrect(PLATE_W, PLATE_W, 0.012), rrect(0.226, 0.226, 0.006),
                               PLATE_BOT + PLATE_T, PLATE_BOT + PLATE_T + 0.008, bevel=0.0008), MAT["alu_black"])
add("Plate", "Plate_TouchGlass", box(-GLASS_W / 2, GLASS_W / 2, -GLASS_W / 2, GLASS_W / 2, PLATE_BOT + PLATE_T,
                                     GLASS_TOP, bevel=0.0004, corner=0.005), MAT["glass"])
rim_top = PLATE_BOT + PLATE_T + 0.008
rs = [xf(cyl(0.0028, rim_top, rim_top + 0.0008, bevel=0.0003), T(0.119 * sx, 0.119 * sy, 0))
      for sx, sy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]]
add("Plate", "Plate_RimScrews", merge(*rs), MAT["steel"])

# ---------------------------------------------------------------------------
# Servo units (X unit authored on +X, Y unit = X unit rotated +90 deg about Z)
# ---------------------------------------------------------------------------
# servo-local frame: origin on shaft axis at the case top face, +Z out of the shaft.
# X unit: servo X -> world X, servo Y -> world Z, servo Z (shaft) -> world -Y.
SERVO_Y_OFF = 0.014   # shaft-face offset so the rod runs in the plane y = 0
TS_X = Matrix(((1, 0, 0, SERVO_R), (0, 0, -1, SERVO_Y_OFF), (0, 1, 0, SERVO_Z), (0, 0, 0, 1)))


def build_unit(tag, U):
    F_HORN = frame(f"F_Horn_{tag}", U @ Vector((SERVO_R, 0, SERVO_Z)), F_BASE)
    F_TIP = frame(f"F_HornTip_{tag}", U @ Vector((ATTACH_R, 0, SERVO_Z)), F_HORN, size=0.01)
    F_ATT = frame(f"F_PlateAttach_{tag}", U @ Vector((ATTACH_R, 0, ATTACH_Z)), F_PLATE, size=0.01)
    F_ROD = frame(f"F_Rod_{tag}", U @ Vector((ATTACH_R, 0, SERVO_Z)), None)
    c = F_ROD.constraints.new("COPY_LOCATION")
    c.target = F_TIP
    c = F_ROD.constraints.new("DAMPED_TRACK")
    c.target = F_ATT
    c.track_axis = "TRACK_Z"
    body(f"Horn_{tag}", F_HORN)
    body(f"Rod_{tag}", F_ROD)
    S = U @ TS_X

    # --- servo case (ground) ---
    add("Base", f"Servo_{tag}_Case", xf(merge(
        box(-0.030, 0.0107, -0.00985, 0.00985, -0.038, -0.003, bevel=0.0010),
        box(-0.028, 0.0087, -0.0086, 0.0086, -0.0035, 0.0, bevel=0.0007),
        box(-0.037, 0.0177, -0.00985, 0.00985, -0.0120, -0.0095, bevel=0.0004),
        cyl(0.0068, 0.0, 0.0025, bevel=0.0003),
        xf(cyl(0.0042, 0.0, 0.0015, bevel=0.0003), T(-0.010, 0, 0))), S), MAT["servo"])
    add("Base", f"Servo_{tag}_Spline", xf(cyl(0.0029, 0.0025, 0.0062, seg=24), S), MAT["brass"])
    add("Base", f"Servo_{tag}_Label", xf(box(-0.027, 0.006, 0.00985, 0.0101, -0.034, -0.016, bevel=0.0002), S),
        MAT["label"])

    # --- servo bracket (ground) ---
    add("Base", f"Bracket_{tag}", xf(merge(
        box(0.030, 0.100, 0.010, 0.062, 0.0, 0.005, bevel=0.0006, corner=0.003),
        xf(ring(rect(0.030, 0.100, 0.005, 0.078), rect(0.0443, 0.0864, 0.0495, 0.0705), 0.026, 0.031,
                bevel=0.0005), M_XZ_Y),
        xf(prism([(0.031, 0.005), (0.060, 0.005), (0.031, 0.045)], 0.030, 0.034, bevel=0.0004), M_YZ_X),
        xf(prism([(0.031, 0.005), (0.060, 0.005), (0.031, 0.045)], 0.096, 0.100, bevel=0.0004), M_YZ_X)), U),
        MAT["alu"])
    hw = []
    for x in (0.041, 0.0897):
        for z in (0.0545, 0.0655):
            hw.append(xf(xf(cyl(0.0021, 0.0, 0.0020, bevel=0.0003), ALONG_Y), T(x, 0.0215, z)))
    for x, y in [(0.038, 0.017), (0.092, 0.017), (0.065, 0.052)]:
        hw.append(xf(cyl(0.0028, 0.005, 0.0065, bevel=0.0004), T(x, y, 0)))
    add("Base", f"Bracket_{tag}_Screws", xf(merge(*hw), U), MAT["steel"])

    # --- servo horn (rotates about the shaft) ---
    add(f"Horn_{tag}", f"Horn_{tag}_Arm", xf(merge(
        cyl(0.0062, 0.0062, 0.0102, bevel=0.0004),
        prism(slot(0.0055, 0.0038, HORN_L), 0.0072, 0.0100, bevel=0.0004)), S), MAT["alu_blue"])
    add(f"Horn_{tag}", f"Horn_{tag}_Hardware", xf(merge(
        cyl(0.0028, 0.0102, 0.0114, bevel=0.0003),
        xf(cyl(0.0018, 0.0100, 0.0190, seg=24), T(HORN_L, 0, 0)),
        xf(cyl(0.0030, 0.0100, 0.0113, seg=32), T(HORN_L, 0, 0)),
        xf(cyl(0.0034, 0.0168, 0.0195, seg=6, bevel=0.0003), T(HORN_L, 0, 0))), S), MAT["steel"])

    # --- push rod with spherical rod ends (rod-local: origin bottom joint, +Z up, eye axis Y) ---
    B = U @ T(ATTACH_R, 0, SERVO_Z)
    eyes, balls, nuts = [], [], []
    for zc, sgn in ((0.0, 1), (ROD_L, -1)):
        eyes.append(xf(tube(0.0040, 0.0064, -0.0025, 0.0025), T(0, 0, zc) @ ALONG_Y))
        eyes.append(cyl(0.0032, *sorted((zc + sgn * 0.0055, zc + sgn * 0.0160)), bevel=0.0003))
        b = sphere(0.0041, 32, 16)
        xf(b, T(0, 0, zc) @ Matrix.Diagonal((1, 0.6, 1, 1)))
        balls.append(b)
        nuts.append(cyl(0.0045, *sorted((zc + sgn * 0.0160, zc + sgn * 0.0205)), seg=6, bevel=0.0004))
    add(f"Rod_{tag}", f"Rod_{tag}_Ends", xf(merge(*eyes), B), MAT["chrome"])
    add(f"Rod_{tag}", f"Rod_{tag}_Balls", xf(merge(*balls), B), MAT["steel"])
    add(f"Rod_{tag}", f"Rod_{tag}_Nuts", xf(merge(*nuts), B), MAT["steel"])
    add(f"Rod_{tag}", f"Rod_{tag}_Shaft", xf(cyl(0.0021, 0.0205, ROD_L - 0.0205, seg=32), B), MAT["steel"])

    # --- plate clevis (rigid with the plate) ---
    add("Plate", f"Clevis_{tag}", xf(merge(
        box(ATTACH_R - 0.010, ATTACH_R + 0.010, -0.010, 0.010, 0.1715, PLATE_BOT, bevel=0.0006),
        xf(prism(ear(0.006, 0.172, ATTACH_Z, u_c=ATTACH_R), 0.0028, 0.0058, bevel=0.0004), M_XZ_Y),
        xf(prism(ear(0.006, 0.172, ATTACH_Z, u_c=ATTACH_R), -0.0058, -0.0028, bevel=0.0004), M_XZ_Y)), U),
        MAT["alu_black"])
    add("Plate", f"Clevis_{tag}_Pin", xf(merge(
        xf(cyl(0.0018, -0.0075, 0.0075, seg=24), ALONG_Y),
        xf(cyl(0.0032, -0.0082, -0.0058, bevel=0.0003), ALONG_Y),
        xf(cyl(0.0034, 0.0058, 0.0085, seg=6, bevel=0.0003), ALONG_Y)), U @ T(ATTACH_R, 0, ATTACH_Z)), MAT["steel"])
    return S


build_unit("X", Matrix.Identity(4))
build_unit("Y", R("Z", 90))

# ---------------------------------------------------------------------------
# Ball
# ---------------------------------------------------------------------------
add("Ball", "Ball_Steel40", xf(sphere(BALL_R, 96, 48), T(*BALL_XY, GLASS_TOP + BALL_R)), MAT["chrome"])

# ---------------------------------------------------------------------------
# Controller board + wiring (ground, visual only for cables)
# ---------------------------------------------------------------------------
PX0, PX1, PY0, PY1 = -0.119, -0.051, -0.1115, -0.0585
PZ = 0.008
add("Base", "Ctrl_PCB", box(PX0, PX1, PY0, PY1, PZ, PZ + 0.0016, bevel=0.0003, corner=0.002), MAT["pcb"])
st = [xf(cyl(0.0028, 0.0, PZ, seg=6, bevel=0.0003), T(x, y, 0))
      for x in (PX0 + 0.0035, PX1 - 0.0035) for y in (PY0 + 0.0035, PY1 - 0.0035)]
add("Base", "Ctrl_Standoffs", merge(*st), MAT["brass"])
top = PZ + 0.0016
add("Base", "Ctrl_Components", merge(
    box(-0.087, -0.075, -0.094, -0.082, top, top + 0.0012, bevel=0.0002),
    box(-0.068, -0.062, -0.100, -0.094, top, top + 0.0010, bevel=0.0002),
    box(PX0 - 0.0025, PX0 + 0.0115, -0.1060, -0.0970, top, top + 0.011, bevel=0.0005),
    box(-0.104, -0.058, PY1 - 0.0028, PY1, top, top + 0.0085, bevel=0.0002),
    box(-0.100, -0.060, PY0, PY0 + 0.0026, top, top + 0.0085, bevel=0.0002),
    xf(cyl(0.0016, top, top + 0.0022, seg=24), T(-0.110, -0.088, 0))), MAT["chip"])
add("Base", "Ctrl_USB", box(PX0 - 0.0035, PX0 + 0.0125, -0.0790, -0.0670, top, top + 0.0105, bevel=0.0004), MAT["steel"])
add("Base", "Ctrl_LED", box(-0.0705, -0.0685, -0.0745, -0.0725, top, top + 0.0008), MAT["led"], smooth=False)

r_c = 0.0011
cable("Base", "Cable_Servo_X", [(0.0440, 0.047, 0.056), (0.040, 0.052, 0.030), (0.036, 0.068, 0.006),
                                (0.020, 0.040, r_c), (0.030, -0.010, r_c), (0.0, -0.050, r_c),
                                (-0.040, -0.048, r_c), (-0.066, -0.046, 0.010), (-0.066, PY1 - 0.0014, top + 0.0095)],
      MAT["cable"], r_c)
cable("Base", "Cable_Servo_Y", [(-0.047, 0.0440, 0.056), (-0.052, 0.040, 0.030), (-0.068, 0.036, 0.006),
                                (-0.070, 0.010, r_c), (-0.072, -0.030, r_c), (-0.078, -0.045, 0.008),
                                (-0.078, PY1 - 0.0014, top + 0.0095)], MAT["cable"], r_c)

# engraved-style label on the base front edge (visual only)
cu = bpy.data.curves.new("Base_Label", "FONT")
cu.body = "BALL  ·  BALANCE  ·  2-DOF"
cu.size = 0.0072
cu.align_x = "CENTER"
cu.align_y = "CENTER"
cu.extrude = 0.00012
lbl = bpy.data.objects.new("Base_Label", cu)
lbl.data.materials.append(MAT["print"])
BODY["Base"][0].objects.link(lbl)
lbl.parent = F_BASE
lbl.location = (0.0, -0.146, 0.00013)
lbl["no_export"] = True

# ---------------------------------------------------------------------------
# Studio: backdrop, lights, cameras, render settings
# ---------------------------------------------------------------------------
def backdrop():
    prof = [(y, -0.022) for y in (-2.5, -1.0, 0.0, 0.6)]
    prof += [(0.6 + 0.6 * math.sin(a), -0.022 + 0.6 * (1 - math.cos(a)))
             for a in [math.pi / 2 * i / 12 for i in range(1, 13)]]
    prof += [(1.2, 2.0)]
    bm = bmesh.new()
    rows = [[bm.verts.new((x, y, z)) for (y, z) in prof] for x in (-3.0, 3.0)]
    for i in range(len(prof) - 1):
        bm.faces.new((rows[0][i], rows[1][i], rows[1][i + 1], rows[0][i + 1]))
    me = bpy.data.meshes.new("Backdrop")
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    ob = bpy.data.objects.new("Backdrop", me)
    ob.rotation_euler = (0, 0, math.radians(45))
    me.materials.append(MAT["backdrop"])
    C_ENV.objects.link(ob)


backdrop()
TARGET = Vector((0.0, 0.0, 0.095))


def aim(ob, target):
    ob.rotation_euler = (target - ob.location).to_track_quat("-Z", "Y").to_euler()


def area_light(name, loc, power, size, color=(1, 1, 1)):
    ld = bpy.data.lights.new(name, "AREA")
    ld.energy = power
    ld.size = size
    ld.color = color
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    aim(ob, TARGET)
    C_ENV.objects.link(ob)


area_light("Key", (-0.55, -0.65, 0.85), 45, 0.6, (1.0, 0.96, 0.90))
area_light("Fill", (0.85, -0.25, 0.35), 10, 0.9, (0.88, 0.94, 1.0))
area_light("Rim", (0.25, 0.75, 0.65), 70, 0.4, (0.90, 0.95, 1.0))
area_light("Top", (0.0, 0.0, 1.1), 14, 0.5)

world = bpy.data.worlds.new("Studio_World")
scene.world = world
try:
    world.use_nodes = True
except Exception:
    pass
bg = world.node_tree.nodes.get("Background") if world.node_tree else None
if bg:
    bg.inputs[0].default_value = (0.10, 0.11, 0.13, 1)
    bg.inputs[1].default_value = 0.25


def camera(name, loc, lens, target=TARGET, fstop=None):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.sensor_width = 36
    ob = bpy.data.objects.new(name, cd)
    ob.location = loc
    aim(ob, target)
    if fstop:
        cd.dof.use_dof = True
        cd.dof.focus_distance = (target - Vector(loc)).length
        cd.dof.aperture_fstop = fstop
    C_ENV.objects.link(ob)
    return ob


CAM_HERO = camera("Cam_Hero", (0.66, -0.76, 0.50), 62, target=Vector((0.0, 0.0, 0.10)), fstop=8.0)
CAM_SIDE = camera("Cam_Detail", (0.34, -0.30, 0.16), 75, target=Vector((0.07, 0.0, 0.105)), fstop=11.0)
CAM_TOP = camera("Cam_Top", (0.0001, -0.0001, 1.25), 60, target=Vector((0, 0, 0.0)))
scene.camera = CAM_HERO

scene.render.engine = "CYCLES"
scene.cycles.samples = 48 if PREVIEW else 256
scene.cycles.use_adaptive_sampling = True
scene.cycles.use_denoising = True
scene.render.resolution_x = 1920
scene.render.resolution_y = 1200
scene.render.resolution_percentage = 50 if PREVIEW else 100
scene.render.film_transparent = False
try:
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
except Exception:
    pass

try:  # use a GPU if one is available
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for dev_type in ("OPTIX", "CUDA", "HIP", "ONEAPI", "METAL"):
        try:
            prefs.compute_device_type = dev_type
        except Exception:
            continue
        prefs.get_devices()
        devs = [d for d in prefs.devices if d.type == dev_type]
        if devs:
            for d in devs:
                d.use = True
            scene.cycles.device = "GPU"
            print(f"[bb] Cycles using {dev_type}: {[d.name for d in devs]}")
            break
except Exception as exc:
    print("[bb] GPU setup skipped:", exc)

bpy.context.view_layer.update()

# ---------------------------------------------------------------------------
# Export: one binary STL per rigid body, in the body frame (m), + MATLAB params
# ---------------------------------------------------------------------------
STL_DIR = os.path.join(ROOT, "cad", "stl")
os.makedirs(STL_DIR, exist_ok=True)
dg = bpy.context.evaluated_depsgraph_get()
summary = {}
for bname, (coll, fr) in BODY.items():
    inv = fr.matrix_world.inverted()
    tris, volume, mass, best = [], 0.0, 0.0, (0.0, None)
    for ob in coll.all_objects:
        if ob.type != "MESH" or ob.get("no_export"):
            continue
        eo = ob.evaluated_get(dg)
        me = eo.to_mesh()
        mw = inv @ ob.matrix_world
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.transform(mw)
        vol = abs(bm.calc_volume(signed=True))
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        for f in bm.faces:
            a, b, c = (v.co.copy() for v in f.verts)
            n = (b - a).cross(c - a)
            tris.append((n.normalized() if n.length > 0 else n, a, b, c))
        bm.free()
        eo.to_mesh_clear()
        mat = ob.material_slots[0].material
        volume += vol
        mass += vol * mat.get("density", 0.0)
        if vol > best[0]:
            best = (vol, mat)
    path = os.path.join(STL_DIR, f"{bname}.stl")
    with open(path, "wb") as f:
        f.write(f"BallBalance {bname} (metres, body frame)".encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    summary[bname] = dict(frame=tuple(FRAME_W[fr.name]), tris=len(tris), mass=mass,
                          color=tuple(best[1].diffuse_color[:3]) if best[1] else (0.5, 0.5, 0.5))
    print(f"[bb] {bname:13s} {len(tris):7d} tris  ~{mass * 1000:7.1f} g  -> {path}")


def mvec(v):
    return "[" + " ".join(f"{0.0 if abs(x) < 1e-7 else x:.6g}" for x in v) + "]"


m = []
m.append("function geo = bb_geometry()")
m.append("%BB_GEOMETRY  Geometry of the ball-on-plate rig (auto-generated by blender/build_ball_balance.py).")
m.append("%   Units: metres, kilograms. World frame: Z up, origin at centre of base top surface.")
m.append("%   All body frames are translation-only from the world frame at the neutral pose.")
m.append("%   STL files are expressed in their body frame. Masses are volume x density estimates.")
m.append("")
m.append(f"geo.pivot        = {mvec((0, 0, PIVOT_Z))};   % universal-joint centre")
m.append("geo.ujoint.axis1 = [1 0 0];   % Base  -> Cross (lower yoke pins)")
m.append("geo.ujoint.axis2 = [0 1 0];   % Cross -> Plate (upper yoke pins)")
m.append(f"geo.horn.length  = {HORN_L:.6g};")
m.append(f"geo.rod.length   = {ROD_L:.6g};")
m.append(f"geo.servoX.shaft = {mvec((SERVO_R, 0, SERVO_Z))};   geo.servoX.axis = [0 -1 0];  % +angle lifts the +X edge")
m.append(f"geo.servoY.shaft = {mvec((0, SERVO_R, SERVO_Z))};   geo.servoY.axis = [1 0 0];   % +angle lifts the +Y edge")
m.append("geo.servoX.hornDir0 = [1 0 0];   geo.servoY.hornDir0 = [0 1 0];   % horn direction at 0 rad")
m.append(f"geo.rodX.bottom  = {mvec((ATTACH_R, 0, SERVO_Z))};   geo.rodX.top = {mvec((ATTACH_R, 0, ATTACH_Z))};")
m.append(f"geo.rodY.bottom  = {mvec((0, ATTACH_R, SERVO_Z))};   geo.rodY.top = {mvec((0, ATTACH_R, ATTACH_Z))};")
m.append(f"geo.plate.width        = {PLATE_W:.6g};")
m.append(f"geo.plate.surfaceWidth = 0.226;   % inside the rim")
m.append(f"geo.plate.surfaceZ     = {GLASS_TOP:.6g};   % rolling surface height (neutral)")
m.append(f"geo.plate.rimHeight    = 0.006;   % rim above rolling surface")
m.append(f"geo.ball.radius  = {BALL_R:.6g};")
m.append(f"geo.ball.center0 = {mvec((*BALL_XY, GLASS_TOP + BALL_R))};")
m.append("")
m.append("here = fileparts(mfilename('fullpath'));")
for bname, s in summary.items():
    key = bname.replace("UJoint_", "")
    m.append(f"geo.body.{key} = struct('frame', {mvec(s['frame'])}, 'mass', {s['mass']:.6g}, ...")
    m.append(f"    'color', {mvec(s['color'])}, 'stl', fullfile(here, 'stl', '{bname}.stl'));")
m.append("end")
with open(os.path.join(ROOT, "cad", "bb_geometry.m"), "w", newline="\n") as f:
    f.write("\n".join(m) + "\n")

# ---------------------------------------------------------------------------
# Save + optional renders
# ---------------------------------------------------------------------------
os.makedirs(os.path.join(ROOT, "blender"), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT, "blender", "ball_balance.blend"))
print("[bb] saved", bpy.data.filepath)

if DO_RENDER:
    out_dir = os.path.join(ROOT, "renders")
    os.makedirs(out_dir, exist_ok=True)
    for cam in (CAM_HERO, CAM_SIDE, CAM_TOP):
        scene.camera = cam
        scene.render.filepath = os.path.join(out_dir, f"{cam.name}{'_preview' if PREVIEW else ''}.png")
        bpy.ops.render.render(write_still=True)
        print("[bb] rendered", scene.render.filepath)
    scene.camera = CAM_HERO
