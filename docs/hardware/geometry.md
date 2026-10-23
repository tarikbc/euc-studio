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

## Suspension

On each side the axle sits in a carriage that slides on a vertical rod inside the channel between the
fore and aft packs. The rods are 30.0 mm (cylinder fit, spread 0.1 mm) at x -0.2, y +63.6 and -64.9 mm,
vertical within 0.15 degrees (top 0.13 degrees rearward). The carriage block's top face is at z
251 mm; the rod is free above it for 108.5 mm up to a ring on the rod (lower face at 359.5 mm: a pipe
O-ring, a travel indicator, see "Rear linkage") and 132 mm up to the frame's top plate (383 mm). Below the axle a frame block
holds the rods (z -52 to -20 mm). The scanned wheel stood unloaded, so the carriage is taken to be at
the bottom of its travel (topped out); the shock's stroke, not the rod, limits the real travel.

What moves with the axle (INFERRED from the layout): the carriage, the axle, the whole motor, rim and
tire, and the shock's bottom eye (below). The pedal hinge brackets sit outside the packs at the axle
height with their plate at the packs' bottom plane (z -29.8 mm), so they are taken to be on the frame
(sprung). The controller board is not visible in the scan.

### The scanned (older) linkage

The scan shows an older suspension system (the owner, 2026-10-20): a shock with a piggyback
reservoir on two 20 mm cross shafts, top (x -139.5, z 367.5 mm) and bottom (x -226.5, z 183.5 mm),
203.5 mm apart, on a sloped rear tray, with plates between the shafts. The owner's photos of
2026-10-22 show none of these parts: where the scan has the rear tray and the bottom shaft, the photo
shows the floor. `geometry.toml` keeps the old positions as `scan_linkage_pivots_xz_m` for the record;
the render model no longer draws them. Only the slider rods, the carriage and the frame stay valid
from the scan.

### Rear linkage (current, from the owner's photos)

The owner cannot measure inside the linkage. The layout comes from the owner's photos: the wheel
lying on its side (2026-10-16), the rear with the side pad cover removed (2026-10-22) and the rear
with the seat removed (2026-10-22). Positions are measured by solving each photo's camera pose from
points that are in the scan (pack rivets, frame bolts, the rear handle's roller bolt): the side view
with the cover off fits 11 scan points to 8 px RMS, about 1 mm at the linkage. The derivation lives
in the private research folder (`research/linkage/`).

- **The rocker (MEASURED).** One black, textured, laser-cut steel plate on each side, outside the
  shock. A retailer's parts diagram (not in the repo) shows the whole plate: a triangle with a notch
  on its rear edge, three windows and four holes. Three of the holes are in the owner's photos: the
  shock's top eye on a cross bar (the red rebound knob beside it), an empty hole 20 mm from it, up
  and forward (likely a second shock position), and the rocker pin 66 mm below the top eye. A
  similarity fit of the diagram's holes to these three (scale error 2 %, residuals under 0.8 mm)
  places the fourth hole, the frame pivot, at (x -66.6, z 398.1 mm) +-3 mm (a fit to hand-picked
  hole centres gives (-67.6, 396.9)). That is the steel pin between two frame blocks seen with the
  seat off (x -45 +-20, z 400 +-15 mm there), so that pin is the rocker's pivot.
- **The shock (MEASURED line).** It runs from its top eye, under the rear roller, forward and down
  toward the carriage: a decal on the shock body places the line 42-52 degrees forward of vertical.
  So its bottom eye, 200 mm away, sits on the carriage just behind the slider channel, above the
  tire (INFERRED, +-10 mm), hidden behind the rear packs in every photo.
- **The link (MEASURED shape, INFERRED lower pin).** The diagram shows a straight bar, 156 mm hole
  to hole at the rocker plate's scale, wider near its upper hole and ending in a pointed tip with a
  small hole past it; the tip shows beside the rocker pin in the photos. In the side photo with the
  cover off, the link's two edges, back-projected onto its plane, run 16 +-2 degrees below
  horizontal forward from the rocker pin. Its lower pin is therefore at about (x -27, z 270 mm)
  +-8 mm, 23 mm above the shock's bottom eye: the link does not share the shock's bottom shaft
  (that would need 25 degrees). The lower pin must ride on the carriage too, or the rocker, its
  pivot and the link would form a rigid triangle.

| Point (full extension) | x mm | z mm | Evidence |
| --- | --- | --- | --- |
| Shock top eye (on the rocker) | -185 | 378 | MEASURED (photo pose, +-3) |
| Rocker pin (link upper hole) | -177 | 313 | MEASURED (photo pose, +-3) |
| Empty hole beside the top eye | -172.5 | 393 | MEASURED (photo pose) |
| Rocker frame pivot (the steel pin) | -66.6 | 398.1 | MEASURED (parts-diagram fit, +-3) |
| Link lower pin, on the carriage | -27 | 270 | INFERRED (link direction and length), +-8 |
| Shock bottom eye, on the carriage | -34 | 247 | INFERRED (shock line, 200 mm), +-10 |

**The leverage: two readings, not used by the physics yet.** The photos place the link's lower
pin but do not show what holds it. A short video from the side, pad cover off, while the seat is
pressed down decides: does the rocker turn?

- **(a) The rocker turns: the link's lower pin rides on the carriage** (what the studio draws). As
  the carriage rises, the shock is pushed toward its top eye while the link turns the rocker a few
  degrees, first so that the top eye rises with it and, once the link passes horizontal (about 50
  mm of travel), back. Drawn as in the table, the ratio is 2.34 axle mm per shock mm at rest, 2.03
  at 45 mm, 1.84 at 75 mm and 1.70 at 93.5 mm (a falling ratio: the wheel gets stiffer through the
  travel), and the 55 mm stroke ends at 107.5 mm of axle travel. A Monte Carlo over the points'
  uncertainties (pivot, top eye and rocker pin +-3 mm, shock line 42-52 degrees, link 14-18
  degrees and 153-159 mm; 4,000 samples) gives a ratio of 2.0-2.5 at rest (median 2.2) and 1.3-2.1
  at 93.5 mm (median 1.6), and the stroke ending at 89-121 mm (median 101); a coarser grid over
  the same ranges spans 1.9-2.7 at rest and 1.0-2.4 at 93.5 mm. The owner's sag then needs about
  4-9 mm of spring preload (median 6, 7 for the drawn points), against the 2 mm assumed so far: with
  2 mm the owner would sag about 25 mm at the axle (the owner reports about 15) and the default 80
  kg rider 23-37 mm (31 for the drawn points) instead of the 18 mm of the constant 1.7.
- **(b) The rocker is fixed: the link's lower pin is on the frame.** The pivot, the link and the
  frame then form a rigid triangle, the rocker is a shock bracket with two positions (the empty
  hole), and only the carriage moves the shock's bottom eye. The ratio is the shock's angle alone
  and constant: 1.52 at the drawn 49 degrees (1.34-1.62 over the photographed 42-52 degrees, 1.70
  at 54 degrees). It matches the owner's sag with little preload: 14 mm at the axle with no
  preload at 1.52, 14 mm with 2 mm at 1.70. At 1.52 the 55 mm stroke ends at 84 mm of axle travel.

The ring on each slider rod (lower face 108.5 mm above the carriage) is a pipe O-ring (the parts
diagram lists pipe O-rings with the slider pipes): a travel indicator, not a stop. The frame's top
plate, 132 mm above the carriage, is the hard limit. The physics keeps the constant `linkage_ratio`
1.7 of `plant.toml` (it fits reading (b) near 54 degrees) until the video. If it shows (a),
`docs/hardware/suspension-linkage.md` holds the candidate `ratio_curve` and preload; the O-ring's
rise after the owner steps off (the sag at the axle) checks either reading.

**The drawn linkage.** The render model draws the rocker plates and links from the diagram's
outlines, placed by the fits above, with the carriage points of the table; the studio poses it as
reading (a), and one flag (`LINKAGE_READING` in `app/src/workspaces/ride/shockLinkage.ts`) switches
it to (b), where the rocker, the link and the link's lower pin stay still. Seen from the solved
camera of the cover-off side photo, its holes, windows, notch and link fall on the photo within a
few millimetres. Over the physics travel (93.5 mm with the constant 1.7) its shock shortens 46.6 mm
where the physics' shock travel is 55 mm: the drawing's shock length differs from the `shock_mm`
channel by up to 8.4 mm. The rocker turns 4.3 degrees at most. The studio poses the drawing from
`suspension_mm`, so all parts stay joined; the shock's length follows the drawing.

**Superseded.** The first estimate (2026-10-22, before the cover-off and seat-off photos) read the
2026-10-16 photos with the scan's old shafts as anchors: a shock leaning 23.5 degrees forward from
the old bottom shaft, a frame pivot forward of the top eye, and a rising curve from 1.75 at rest to
2.38 at 103.5 mm. The new photos contradict its positions (the top eye sits 40 mm further back and
the old bottom shaft is gone), so it is not used. Its scenario runs (no verdict changed against the
constant 1.7) say nothing about the real linkage. The second estimate, before the parts diagram,
had the link on the shock's bottom shaft and the pivot hidden: its Monte Carlo (pivots between the
rear packs, ratio 2.1-2.7 at rest, 41-55 mm of stroke used) is superseded by the located pivot and
link pin. `docs/hardware/suspension-linkage.md` records the readings considered and why each was
kept or dropped.

## Pad covers

The side pad covers are not in the scan (the scanned packs show their bare outer faces with rivet
rows). The render model sculpts them from the owner's two photos of a removed cover (2026-10-22:
one from above, one low from its front end), sizes and places them from the owner's side photo of
the wheel with the cover mounted (2026-10-22), and checks them against a retailer's parts diagram
(not in the repo; its left and right side pads). The owner's words: the pads start below the top
of the battery case.

- **Shape (INFERRED).** Twenty points of the cover (the hole, the lettering block's top corners,
  the three screw heads, groove ends, the round block's top, and six points where the cover touches
  the floor) were picked in both photos of the removed cover. A bundle adjustment with the phone's
  known focal length (48 mm equivalent) solves both cameras and the points together: 5.5 px RMS in
  the top photo and 3.1 px in the low one. The floor is the cover's mounting face, so each point's
  height above the floor is how far it stands out from the packs. Outlines come from the top
  photo, projected onto the plane of their face.
- **What it is.** A C of moulded black plastic, open at the back with 3.5 mm walls (ESTIMATE): a
  lettering block at the front top (its lettered face slopes from its top face to the mounting
  face; the hole in its top face; the maker's lettering is left out); an upper arm running back
  as a bar that ramps up into a raised end block (the upper arm's rear leg); a spine down from the
  lettering block; the lower arm, with three diagonal grooves, its front and rear ends sloping down
  to the packs; on the lower arm a slotted block by the spine (two pockets, one with a screw in its
  floor, and a shallow tray at its front) and a round-topped block at the rear end, traced on the
  mounted photo; three steel screw heads. One smooth moulding: the joined shape is smoothed (a 4 mm
  Gaussian on its distance field), so every edge is rounded and the elements blend with 11 mm
  fillets; the pockets, grooves and the hole are cut after.
- **Size and place (MEASURED from the mounted photo).** The camera of the side photo is solved
  from points that are in the scan or the model: pack rivets, the rocker's top-eye bolt and empty
  hole, the rear handle's roller bolt and its three windows (6.9 px RMS, with the phone's 24 mm
  equivalent focal length). The cover's own points (the hole, the three screws, the groove ends,
  the round block's top) then fix its place, scale and in-plane proportions (4.5 px RMS): 366 +-8
  mm long and 299 +-6 mm tall, x -182 to +184 mm, z 55 to 354 mm (+-6 mm), turned 2.6 degrees nose
  up. Its top is 16 mm below the packs' top (370 mm), as the owner says; its bottom edge is 43 mm
  above the deployed pedals' top. Against the removed-cover photos the mounted cover is 2.4 %
  shorter across (up-down), within their error.
- **On the packs (the owner: the pads hug the battery).** The cover's back is cut to the packs'
  shape: flush on their outer faces (scan: |y| 101.05 mm left, 102.05 mm right; each cover sits on
  its own side's face) and, where the cover runs past the packs' front and rear ends, wrapping round
  those end faces 35 mm deep (to |y| 66 mm), as the mounted photo shows. The depth is
  where the arms' undersides sit on the low photo of the removed cover (the lower of the two, so
  both arms touch). The top edge stays 16 mm below the packs' top, so the cover does not wrap
  there. **Contact, measured** on the exported LOD0 against the drawn packs (boxes on the scan's
  measured faces, table above): along the cover's outline over the packs' outer faces (3 mm cells,
  the cover's innermost point in each) the gap is median 0.2 mm, 95th percentile 1.5-1.7 mm. The
  larger values are the lettering block's hole (a hole, not a gap) and single cells that hold only
  a sliver of a sloped edge (up to 9 mm; they differ between the two sides, which are the same
  mesh). The wrap faces are 0.6 mm from the packs' front and rear end faces (medians; 95th
  percentile 0.9 and 2.7 mm). The deepest overlap is 0.09 mm. The walls are hollow from the back, so a horizontal section through
  an arm shows only its outer wall; the arm's top and bottom walls stand on the pack.
- **How far it stands out (INFERRED at that scale, +-20 %).** From the packs' outer faces: arms
  16-18 mm, spine 25-35 mm, slotted block 45-47 mm, lettering block 47-51 mm, round block 52 mm, end
  block 60 mm; outer faces up to |y| 162 mm, inside the deployed pedals (|y| 247 mm). The rounding
  takes 2-5 mm off the outline's corners (x -179 to +179 mm, z 56 to 352 mm, inside the photo's
  +-6 mm).
- **Calf contact faces (leg-pad contact).** The outer faces where the rider's
  legs can rest are listed in `geometry.toml` `[pad_covers] contact_faces` (from the element
  outlines above): the lettering block's top, the upper arm, the end block's top, the round
  block's top and the lower arm.
- **Physics.** The drawn covers have no mass in `plant.toml`. Since 2026-10-22 the physics builds one contact box
  per contact face, from the packs out to the face (`pad_<face>_<side>`); only the lateral rider's legs
  (`articulated_lateral_v1`) touch them ([body.md](../models/body.md)).

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
(8 % higher) and the contact geometry.

**Revised 2026-10-23.** The table assumes that the firmware's 0.0505 m per electrical turn is the
true rolling circumference per pole pair. The owner's ride log (2026-10-17) shows that it is not: GPS
reads 0.905 of the displayed speed over 54 steady rows. The plant keeps this tire's 0.2251 m and now
has 31 pole pairs, so the firmware shows 1.107 times the true speed, as on the owner's wheel
(`docs/models/electrical.md`, "Calibration against the owner's ride log"). The "16 x 3 inch (published)" note in the electrical model
conflicts with the tire on the scanned wheel; the owner's photo confirms the 3.00-12 tire.

## Pedals

The CNC decks were off the wheel for the scan; only the hinge brackets and pins are in it. The
model's deck, hinge rod and bracket follow the owner's tape photo of a deployed deck (2026-10-16) and
a retailer's parts diagram (not in the repo; its pedal set, pedal rods and pedal brackets). Left
and right are mirror images.

- **Length: 300 mm, MEASURED +-10 mm.** The owner's tape across the deck reads 2.6 to 32.7 cm end to
  end; the tape lies above the deck, so the reading carries about 10 mm of parallax.
- **Outline and width: 138 mm hinge edge to outer edge, INFERRED +-7 mm.** Traced from the diagram,
  where the deck is 277 px long; at the tape's 300 mm that is 1.06 mm per px. The diagram is taken
  slightly in perspective (the near end 3 % wider), and the deck is symmetric front to back (the
  diagram's cell centres repeat within 2 px about the notch, and the tape photo shows matching
  ends), so each traced edge is the mean of the two halves. The deck is full width over its middle
  third and rounds off toward both ends, also on the hinge side.
- **Cut-outs: INFERRED.** Hexagons with their points along the deck's length, on the diagram's
  lattice: columns 51.6, 90.1 and 128.6 mm from the middle, cells 46.1 mm apart across the deck,
  38.5 mm across the flats (7.6 mm webs) with 5 mm corner radii (the tape photo's soft corners),
  cut back by a 7.5 mm rim along the outline. Eight cells per
  half, the outer ones partial, as in both pictures.
- **Solid strips: INFERRED.** A plain centre strip 62 mm long (it carries the maker's lettering on
  the real deck; the model leaves it plain) and a solid strip 25 mm wide along the hinge edge.
- **Edge and finish.** Black anodised faces with a bright machined side wall and top chamfer all
  round (the tape photo shows a silver rim); steel grip pins on the web junctions (28 per deck;
  the diagram shows them at most junctions).
- **Hinge and notch.** MEASURED: the 6 mm pin runs fore-aft at y +-114.8 mm, z +7.5 mm. The notch
  must clear the scanned bracket (46 mm long, its outer face 20.3 mm past the pin), so it is 50 mm
  long and reaches 21 mm past the pin. The diagram's notch is 45-56 mm long but only about 20 mm deep
  from the hinge edge, which would not clear the scanned bracket; the bracket fit decides. The
  hinge rod is drawn 190 mm long (ESTIMATE: that is its length at the deck's diagram scale; 125-200
  mm if the rods are pictured at another scale).
- **Thickness: 9 mm, ESTIMATE +-2 mm** (no picture shows the edge square on). The plate is centred
  on the pin, so its top is at z +12 mm.
- **Bracket.** The scanned hanger's envelope (MEASURED: x -22.1 to 23.5, y 100.7 to 135.5, z -31.1
  to 10.5 mm) drawn clean: a black block holding the rod, an outer web, and a bright flange at the
  packs' bottom plane with the diagram's three screw holes (layout ESTIMATE).
- **Fold.** Each deck is its own node with its origin on the pin, so the app folds it about x: the
  left deck up with +90 degrees, the right with -90 degrees (glTF axes).

**Physics values not changed.** The collision boxes `pedal_left` and `pedal_right` keep 290 x 135 x
9 mm centred at y +-176.3 mm, and `plant.toml` keeps `pedal_below_axle_m` -0.012. With the drawn
deck they would become 300 x 138.3 x 9 mm (half sizes 0.150, 0.0692, 0.0045) centred at y +-177.95
mm: 10 mm longer, 3.3 mm wider, the centre (where the feet stand) 1.65 mm further out. The deck's
height does not change (same thickness and pin), so `pedal_below_axle_m` stays.

## Collision shapes

`[collision]` gives primitives in the MJCF frame; the physics uses them:

- **Tire.** A torus with a 58 mm tube at 171.6 mm from the axle has the right outer radius and the
  real crown curvature; it stays within 1.5 mm of the tread envelope up to +-37 mm off the centre
  plane, about 40 degrees of lean. MuJoCo has no torus geom: the physics builds `tire_torus` as one
  convex mesh of the tube's outer half (360 points around, 13 across), soft against the ground as a
  tire at 25 psi (docs/models/body.md, "Tire"). Until 2026-10 it was a ring of 48 capsules, whose
  0.37 mm ripple per segment shook the shell; the `mujoco.sdf.torus` plugin is the other way. An
  earlier ellipsoid had the right outer radius but a 7 mm crown radius instead of 58 mm. A second
  torus (40.7 mm tube) keeps the true width with a sharper crown.
- **Packs.** One box per side over both packs and the slider channel, bottom at the lip (-41.5 mm).
- **Top and front.** A box over the seat and the top frame; a box over the handle, headlight and
  display chamfer.
- **Rear roller and shock.** Capsules along their axes.
- **Pedals.** The bracket boxes are measured; the deck boxes keep the earlier 290 x 135 mm (see
  "Pedals").

## The render model

`app/public/models/begode-extreme/` holds three levels of detail (LOD0 350k triangles 2.9 MB, LOD1
146k 1.4 MB, LOD2 54k 0.48 MB, meshopt compressed). Its README lists the nodes. How it is made from
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
   stub, pedal decks (above), and the rear roller: its tube is redrawn as two smoked grey sleeves
   around a black centre band with the BLE status LED; the scanned end caps become the two speakers.
6. **Packs, shock and rear handle.** The four battery packs are modelled as boxes with 4 mm rounded
   edges and 8 mm chamfers on the faces measured above, with the rivet rows read on a height map of
   the side (four groups of five near each outer end). The rear shock is a DNM coil-over without
   reservoir: **200 mm eye to eye and 750 lb/in, MEASURED by the owner**; a 56 mm x 115 mm spring
   (DNM size chart for 200 mm), 8 mm x 24 mm eyelets, threaded body with a slotted collar at the
   bottom eye, red rebound knob at the top eye; stroke ESTIMATE (55 mm). The shock and the **rear
   linkage** are drawn to the owner's photos of 2026-10-22 and the parts diagram (see "Rear
   linkage" above): per side a rocker plate with the diagram's outline, three windows and four
   holes; a link with the diagram's outline (wider near the rocker pin, the pointed tip and its
   small hole past it) to its own carriage pin; cross bars at the top eye and the rocker pin (black
   tubes with spacers either side of the shock eye, chrome button heads with washers); the plates
   sit at |y| 46.5-51 mm (the rear photo's camera pose fits 50-54 mm for their outer faces); the
   carriage's clevis and bottom shaft for the shock, and the link's lower pin with a plate per side
   (its own node, so the studio can hold it on the carriage or on the frame; hidden in every photo,
   ESTIMATE); and the pivot pin between two frame blocks.
   Nothing of the scan's older system is drawn: its shafts, plates and rear tray are removed. Each
   moving part is its own node (README), and the studio poses them from the Suspension channel.
   The **rear handle** is the orange CNC U-handle of the photos: each arm runs from a foot at the
   frame top (x -113 mm) up through the roller axis to the grip bolt. MEASURED on the cover-off
   photo (camera pose from 11 scan points; edges traced and back-projected onto the arm's outer
   face, +-2 mm): the lower part (the foot, three windows) and the upper part, whose top runs
   nearly level (z 480-485 mm) to a 9.5 mm round end at the grip and whose lower edge drops in a
   dog-leg to the roller; the two upper windows are pocketed rims (3 mm deep in two steps, like the
   machined terraces) around smaller see-through openings, the window by the foot a pocketed slot;
   a dog-leg blind pocket above the roller. The grip bolt is 62.8 mm past the roller axis and the
   black anodised grip bar is 20 mm across (the photo; it was a 26 mm ESTIMATE). Socket-head cap
   screws at the roller and the grip stand 3.8 mm proud of the arm, as the scan shows at the
   roller (head 10 mm across, hex socket). The outer face at |y| 58.7 mm is on the scan; the 6 mm
   thickness is an ESTIMATE. The physics' collision solid for the grip
   (`[collision] rear_handle_grip`) keeps the earlier estimate, about 31 mm higher than the
   photo's grip; the rider rarely touches it. The scanned faces these models
   replace are removed; what remains of the scan there (brackets, a sloped base plate) is cleaned
   (loose flaps under 150 mm2 dropped, holes closed, smoothed). The roller is cut out of the seat
   before decimation, and its end caps (speakers) are modelled from their measured profile. The
   **side pad covers** are sculpted from the owner's photos and placed from the mounted photo ("Pad
   covers" above), grained like the shell. The **front handle** is of the rear handle's orange CNC
   family (the owner): the parts diagram's front handle part and black crossbar. The diagram's
   part is placed by five features of the mounted side photo, back-projected with its solved
   camera, and its crossbar hole on the scan's tube: residuals 0.9-2.8 mm; its outline then
   matches the scan's side plate within 3 mm. Per side an L-shaped plate: a lower body at the
   front, a foot to the rear bolted to the frame top by 2 x 2 socket-head screws, and an arm that
   rises from the body and leans back to the crossbar; three pocketed windows (3 mm) with
   see-through centres (a rounded triangle and a long slot in the arm, a slot in the body), a small
   slot and a hole with a screw in the body. Faces from the scan: |y| 59.5-67.5 mm, the foot raised
   to 74 mm. The foot screws' heads are in the scan (10 mm across, 6.2-7.7 mm tall, places within
   0.5 mm); they are redrawn there as steel socket heads and the scanned ones are removed. The
   crossbar stays the scan's 20 mm tube, black like the rear grip. How it differs
   from the rear handle: one L-shaped plate per side bolted by a 2 x 2 foot (the rear arm: one
   frame bolt and the roller axis); 8 mm thick (the rear 6 mm); 119 mm between the plates' inner
   faces (the rear arms 105 mm); a triangle and two slots (the rear: two four-sided windows, a slot
   and a blind dog-leg pocket); six screws per side (the rear: two drawn, at the roller and the
   grip). Both: orange anodised aluminium, 3 mm pockets, socket-head screws, a black 20 mm bar at
   z 474 mm.
   The **internal fender** (the parts diagram's "Internal Fender") is in the scan: thin arcs
   between the packs that follow the tyre's top 4-35 mm above its crown (front x 57 to 142 mm, rear
   x -128 to -71 mm). The scan is at full extension. With at least 93.5 mm of axle travel, a fender
   fixed to the frame that close would meet the tyre after 12-20 mm of travel, so it rides on the
   carriage: it is its own node and moves with the axle. A cable bundle runs from the controller
   down to the fender's front end; its low part lies in the tyre's path, so it is its own node with
   a travel morph target (its vertices follow the carriage fully inside the tyre's path, less up to
   60 mm away: the cable flexes). The axle stubs ride on the carriage too. Tyre clearance with the
   tyre moved up the rods as the physics moves it (LOD0, smallest distance from each part to the
   moved tyre): fender 8.9 mm at every travel (it moves with the tyre), cable 20.9 mm, the rest of
   the shell 14.1 mm at full extension, 3.0 mm at the sag (18 mm) and 9.4 mm at full travel (93.5
   mm), pad covers 25 mm; none touches it. The diagram's rear fender is not in the scan and is not
   drawn.
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
   flange ring, textured matte black linkage plates, smoked grey tail-light sleeves (they glow with
   the LEDs in the studio), orange front and rear handle arms with black 20 mm bars and bright
   brushed-steel socket-head screws (half metallic, so they read silver in the studio and in
   daylight, as in the photos), black anodised pedals with steel pins,
   smoked display window. Ambient occlusion is baked into the vertex colours. The anodised orange
   is set from the owner's photo (mean sRGB 184, 91, 47 on the handle). Under the studio's lights
   (neutral tone mapping, neutral rim light) the handle, the chrome bolt heads and the shell render
   within 26, 51 and 13 sRGB units of the photo's mean colours in the same view; with the earlier
   ACES tone mapping and warm lights they were 91, 206 and 43 units off (the handle red, the chrome
   copper, the shell brown).

## Open

What the owner can measure to close the remaining gaps (centre to centre, in mm):

- **Rocker linkage.** A short video from the side with the pad cover off while pressing the seat
  down: whether the rocker plates turn about the steel pin and which way (reading (a) or (b)). And
  the sag: push a slider pipe's O-ring down to its seal, stand on the wheel, step off and measure
  how far the O-ring rose; or the spring preload (how far the collar is wound down from the
  spring's free length). With both, the leverage becomes a measured curve.
- **Pad covers.** A tape length of a removed cover would check the photo-measured 366 mm.
- **Shock stroke.** Printed on the shock or in its listing (55 mm is an ESTIMATE, 50-57 mm range).
- **Pedals.** The deck's width at the middle (hinge edge to outer edge) and its thickness, with a
  tape; and the hinge rod's length if it comes out.
- **Rear handle.** Distance from the roller axis to the grip centre (the photos give 62.8 mm).
- **Front handle.** A clear side photo of the front handle (the diagram and the mounted photo
  place it now).
- **Rear fender.** Whether the wheel has the diagram's rear fender (it is not in the scan).
- **Controller.** Where the controller board sits (for the IMU height above the axle).
