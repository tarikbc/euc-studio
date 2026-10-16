# Body geometry (Begode Extreme)

`profiles/begode-extreme/geometry.toml` holds the size and layout of the wheel's body: tire, rim,
shell, seat, handle, display window, lights, pedal mounts and the collision shapes proposed for the
physics model. The numbers come from a 3D scan of a real Extreme, made by Tarik Caramanico. The raw
scan and the intermediate files stay in the private research folder; the cleaned render model is
part of this repo (`app/public/models/begode-extreme/`, see "The render model" below).

Evidence tags in the TOML: **MEASURED** (fitted or read on the scan), **INFERRED** (measurements plus
one stated assumption), **ESTIMATE** (the part is not in the scan), **DEFAULT** (kept from
`plant.toml` or the MJCF).

## The scan

- Binary STL from a GOM structured-light scanner, 1,610,052 triangles, millimetres, "GOM aligned".
  Vertex spacing is about 1 mm; flat panels are meshed coarser (edges up to 20 mm).
- **No colour.** The header carries `COLOR=` with RGBA 255,255,255,255 (the Materialise default
  colour) and every face has the attribute value `0x8000`: bit 15 set, RGB bits zero. In the
  Materialise convention bit 15 means "use the default colour", in the VisCAM convention it means
  "this face is black". Either way all faces have the same colour, so there is nothing to keep.
- The wheel stood on its tire; the scan stops 89 mm above the floor, so the contact patch is
  missing. The pedals were off the wheel (only their hinge brackets are there).
- After welding, the mesh splits into 198 connected pieces. The largest is the body (frame, packs,
  seat, handle, lights); the tire comes out as two halves and the motor's side covers and a rim
  flange as separate pieces. That split is what makes the wheel easy to isolate.

## The frame

Same as the MJCF: x forward, y left, z up, metres, origin on the axle centre in the wheel's
mid-plane.

1. **Rough axle.** A circle fit to the tread's outer envelope (the five farthest points in each
   1-degree bin around a guessed centre) gives the axle to within a few millimetres.
2. **Wheel faces.** Faces outside the body piece, within 46 mm of the wheel plane and inside the
   tread radius.
3. **Axle line.** On a surface of revolution every normal line crosses the axis. The axis (point and
   direction) minimises the sum of `((p - c) . (a x n))^2` over the motor covers and rim faces (radius
   under 147 mm, 95,000 faces, soft-L1 loss). Median residual 0.42 mm. Fitting the tire sidewalls
   instead moves the centre by about 2 mm (the tire is not a perfect solid of revolution), so the
   rigid motor is the reference.
4. **Mid-plane.** Halfway between the outer faces of the two sidewalls at their widest (they sit at
   +-40.7 mm from it).
5. **Up.** The normal of the shell's flat bottom face (1,800 faces, plane fit RMS 0.03 mm). The
   scanner's alignment already had z normal to it to 0.007 degrees.
6. **Forward.** The end with the four-lens headlight, the carry handle and the display window; the
   shock and its spring sit at the other end. The display window's orientation agrees: the display
   doc records the top of the digits pointing forward.

The axle ends up 0.13 degrees yawed and 0.25 degrees rolled against the shell, which is play in the
slider and bearings of the scanned wheel, not something to model. The tread's knob tips lie at
229.0-230.4 mm from this axle at every angle the scan covers, which checks the centre.

## Measurements and methods

| Quantity | Value | Method | Confidence |
|---|---|---|---|
| Tire size | 3.00-12, code J802-02 | moulded text on the sidewall, read on a height map of the unwrapped sidewall; the owner's photo confirms it | high |
| Tread | 15 repeats of 24 degrees, staggered rectangular knobs | autocorrelation of the crown band (0.99 at 15) | high |
| Tire outer diameter | 459.3 mm | median of the knob-tip radius over 51 bins of 4 degrees, about the fitted axle | +-1 mm, unloaded |
| Tread base radius | 225.0 mm | lower peak of the radius histogram in the tread centre band | +-0.5 mm |
| Tire width | 81.4 mm | sidewall extremes at radius 170-205 mm | +-0.5 mm |
| Crown curvature | 58 mm | circle fit to the tread envelope in the axial-radial plane | +-3 mm |
| Rim | bead 304.8 mm (12 in), flange 320 mm, flanges 61.4 mm apart | sidewall end and flange faces in axial-radial sections | +-3 mm |
| Motor width | 80.5 mm | the two side-cover planes | +-0.5 mm |
| Axle height | 229.6 mm unloaded | equals the free tire radius (no floor in the scan) | +-1 mm |
| Packs | x +-124.3, y 44.3-101.3 (left) and 45.8-102.3 (right), z -29.8 to 370.3 mm | area-weighted histograms of axis-aligned faces | +-0.5 mm |
| Lowest body point | z -51.8 mm | axle block under the slider channel | +-0.5 mm |
| Seat top | z 469.5 mm, 699 mm above the ground | 99.5th percentile of the seat cover on the centre line | +-1 mm |
| Handle tube | centre x 132.3, z 473.8 mm, diameter 20.0 mm, 138 mm long | circle fit in the x-z plane (median residual 0.02 mm) | +-0.5 mm |
| Display window | flat at z 438.3 mm, x 41-127, y -54 to 52 mm, tilt 0.15 degrees | plane fit to the flat faces in front of the seat | window +-1 mm, board place +-10 mm |
| Headlight | 4 lenses of 22 mm at y -45.7, -15.7, 14.3, 44.3 mm, z 390.5, face at x 175.1 | 2 mm lens recesses in the bar's front face | +-0.5 mm |
| Pedal pivots | pin along x at y +-114.8, z +7.5 mm, 6 mm diameter | Hough vote of normal lines in the hinge brackets | +-1 mm |
| Pedal deck | 290 x 135 x 9 mm, top at z +12 mm (242 mm above the ground) | not in the scan; see "Pedals" below | estimate: length +-30, width +-15, top +-5 mm |
| Rear roller | tube along y at x -186.8, z 449.1 mm, radius 26.1 mm; end collars from y +-37.5 mm; speaker grille faces at y +-50.5 mm (annulus r 12-22 mm) | circle fit (0.67 mm); radius profile along y; recessed end faces with radial windows | +-1 mm |
| Overall | 479 x 270 x 715 mm (length x width x height above the ground) | body bounding box plus the tread | +-1 mm |

What the scan cannot give:

- **Pedal decks.** Estimated from photos (below). One measured pedal replaces the estimate.
- **Inside the rear roller.** The owner confirms what the tube holds: the controller's Bluetooth data
  board (a blue status LED at the top of the centre band), the two RGB tail-light LED rings (the
  translucent halves glow orange-red on each side of a black centre band) and a speaker in each end
  cap, driven by a separate Bluetooth audio device. The scan shows the tube, the collars and the
  grilles, but no seam for the band: its width (15.6 mm) and the LED spot come from the owner's video
  (`lights.toml`).
- **IMU.** The controller board is inside the body. The findings default (0.15 m above the axle)
  stays. If the board sits in the top box under the seat (z 0.37-0.44 m) the IMU is about 0.4 m above
  the axle, which changes the tangential accelerations it feels; a photo of the open wheel settles it.
- **Display board.** The window is measured; the board is assumed centred under it.

## Rolling radius and the electrical model

`docs/models/electrical.md` takes the rolling radius from the firmware's 0.0505 m per electrical turn
and 26 pole pairs: 0.2090 m, a 16.5 inch tire. The scanned tire is a **3.00-12** with knob tips at
0.2296 m, 20.6 mm (9.9 %) larger. Only one whole pole-pair count fits it:

| Pole pairs | Rolling radius (p x 0.0505 / 2 pi) | Against the free radius 0.2296 m |
|---|---|---|
| 26 | 0.2090 m | 20.6 mm smaller: impossible for this tire |
| 28 | 0.2251 m | 4.5 mm smaller: the usual squash of a loaded knobby tire |
| 30 | 0.2411 m | larger than the free tire: impossible |

So the scan points to **28 pole pairs and a rolling radius of 0.2251 m** (`geometry.toml` records both
as INFERRED). The firmware does not care: its speed and odometer come from electrical turns times
0.0505 m, and the force at the ground per amp, 1.5 psi I / (0.0505 / 2 pi), does not depend on the
pole pairs either, so the motor flux from the no-load speed stays the same. What changes is the
mechanics: the wheel's size, its spin speed for a given road speed (7 % lower), the torque per amp
(8 % higher) and the contact geometry. The "16 x 3 inch (published)" note in the electrical model
conflicts with the tire on the scanned wheel; the owner's photo confirms the 3.00-12 tire.

## Pedals

**ESTIMATE, to be updated from the owner's measurements** (pedal height, size and position; the owner
also says the drawn design does not match his pedals). The CNC decks were off the wheel for the scan. The model draws our own deck in their style: a black
anodised aluminium plate with a D-shaped outer edge and a bright chamfer, hexagonal through-cut-outs in
the front and rear thirds, a plain solid band in the middle, gold grip pins on the cell junctions
(26 per deck) and a polished hinge pin. Left and right are mirror images. The sizes are estimates:

- **Hinge.** Measured: the 6 mm pin runs fore-aft at y +-114.8 mm, z +7.5 mm. The deck's notch must
  clear the scanned bracket (46 mm long, its outer face 20.3 mm past the pin), so the notch is 50 mm
  long and 21 mm deep. The plate (9 mm) is centred on the pin, so its top is at z +12 mm.
- **Length and width.** Shop photos give the proportions: the hinge notch is 0.16 of the deck length
  and the width (hinge edge to outer edge) is 0.47 of it. A 50 mm notch makes the deck about 300 mm
  long; with a little margin around the bracket, 290 x 135 mm. Uncertainty: +-30 mm length, +-15 mm
  width, +-5 mm deck height. One tape measure on a real pedal settles it.
- **Fold.** Each deck is its own node with its origin on the pin, so the app folds it about x: the
  left deck up with +90 degrees, the right with -90 degrees (glTF axes).

The collision boxes `pedal_left` and `pedal_right` follow these sizes: 290 x 135 x 9 mm, centred on
the pins at y +-176.3 mm.

## Collision shapes

`[collision]` proposes primitives in the MJCF frame:

- **Tire.** A torus with a 58 mm tube at 171.6 mm from the axle has the right outer radius and the
  real crown curvature; it stays within 1.5 mm of the tread envelope up to +-37 mm off the centre
  plane, about 40 degrees of lean. MuJoCo has no torus geom: a ring of 48 capsules (0.37 mm ripple per
  segment) or the `mujoco.sdf.torus` plugin. The current ellipsoid has the right outer radius but a
  7 mm crown radius instead of 58 mm, which matters only once the model leans in turns. A second
  torus (40.7 mm tube) keeps the true width with a sharper crown.
- **Packs.** One box per side over both packs and the slider channel, bottom at the lip (-41.5 mm).
- **Top and front.** A box over the seat and the top frame; a box over the handle, headlight and
  display chamfer.
- **Rear roller and shock.** Capsules along their axes.
- **Pedals.** The bracket boxes are measured; the decks use the estimated sizes above.

## The render model

`app/public/models/begode-extreme/` holds three levels of detail (LOD0 300k triangles 2.5 MB, LOD1
114k 1.2 MB, LOD2 40k 0.37 MB, meshopt compressed). Its README lists the nodes. How it is made from
the scan (the tools live in the private research folder):

1. **Parts.** Simple rules split the scan: tire (wheel pieces that reach the tread), hub, seat
   (above 405 mm between the rear roller and the handle), display window (the flat faces at 438 mm),
   handle, headlight, shock, pedal mounts and shell. Pieces under 25 faces are dropped.
2. **Body clean-up.** Duplicate and null faces, non-manifold edges and holes up to 120 border edges;
   feature-preserving smoothing (normals, then vertices). On the hard parts (shell, handle, headlight,
   pedal mounts) 63 planes are found from the face normals and offsets, and every face within 8 degrees
   and 0.7 mm of one goes onto it, so panels are flat and their edges straight (moves under 0.7 mm).
   The seat and shock stay curved. The handle tube is redrawn on its fitted cylinder.
3. **Tire.** Rebuilt as a closed ring: the scanned tire becomes a height field around a section centre
   at 180 mm, its tread repeat (15 x 24 degrees) is found by autocorrelation, all scanned repeats are
   merged by median into one clean pitch (0.15 mm from the scan) and tiled round the whole wheel. This
   fills the part under the floor and inside the shell and removes noise. The sidewalls are made
   smooth surfaces of revolution (the size text goes).
4. **Hub and rim.** Modelled from the measured profile: matte black motor covers with a ring of socket
   screws, and the rim flange as an orange ring (the owner's photo). The scan had holes there.
5. **Modelled parts.** Display window (flat, with UVs for the T16-007 panel), headlight lenses, axle
   stub, pedal decks (above), and the rear roller: its tube is redrawn as two translucent sleeves
   around a black centre band with the BLE status LED; the scanned end caps become the two speakers.
6. **Packs, shock and rear handle.** The four battery packs are modelled as boxes with 4 mm rounded
   edges and 8 mm chamfers on the faces measured above, with the rivet rows read on a height map of
   the side (four groups of five near each outer end). The rear shock is a **DNM-type coil-over
   without reservoir, per owner photos 2026-10-16; spring rate unknown** (the 2026-10 scan shows an
   older shock with a piggyback reservoir). Its eyes sit on the scan's cross shafts (top x -140.5,
   z 368.1 mm; bottom about x -224, z 182 mm, +-5 mm); the rest is scaled from the photo by the
   204 mm eye-to-eye length and is ESTIMATE: threaded body (29 mm) with a ridged end cap at the
   bottom eye, slotted preload collar, black spring (52 mm outside, 8 mm wire, 7 coils), chrome
   shaft (12 mm) and a red rebound knob at the top eye. The link plates and cross shafts are the
   scan's. The **rear handle** is the orange CNC U-handle of the photos: the lower part of each arm
   (frame bolt at x -115, z 415 mm to the roller axis, three windows, outer face at |y| 58.7 mm) is
   in the scan as the "roller arms"; the part above the roller (85 mm, bent 15 degrees up) and the
   black grip bar (26 mm) are ESTIMATES scaled with the roller. The scanned faces these models
   replace are removed; what remains of the scan there (brackets, a sloped base plate) is cleaned
   (loose flaps under 150 mm2 dropped, holes closed, smoothed). The roller is cut out of the seat
   before decimation, and its end caps (speakers) are modelled from their measured profile.
7. **No trademarks.** The brand emblem pressed into the seat top is flattened (a quadratic surface
   fitted to the plain ring around it), the pedal band stays plain, and the shock carries no brand
   marks.
8. **Levels of detail.** Hard parts: vertex clustering, smoothing, quadric collapse, then back onto
   the planes. Curved parts (seat, scanned brackets): a 1 mm clustering pass, smoothing, quadric
   collapse with optimal placement, smoothing again. Quadric collapse alone on a raw scan with
   100,000 open border edges builds slivers hundreds of mm long. Loose scanned shards (under 60, 150
   and 400 mm2 for LOD0, LOD1, LOD2) are dropped.
9. **Looks.** The scan has no colour. Materials follow the owner's photos: textured dark-grey shell
   (a tileable grain normal map on LOD0 and LOD1), black rubber tire, matte black motor covers, orange
   flange ring, satin aluminium handle tube, black anodised pedals with gold pins, smoked display
   window. Ambient occlusion is baked into the vertex colours.
