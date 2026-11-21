# Ready-to-install tracks

Each folder here is a complete Assetto Corsa track. To install, copy the folder
(for example `ar12_gentry/`) into

```
<Steam>/steamapps/common/assettocorsa/content/tracks/
```

so you end up with `.../content/tracks/ar12_gentry/ar12_gentry.kn5`. If you're updating
an older copy, delete the old folder first rather than copying over it. Then pick it in
Content Manager or the AC launcher. You don't need Blender or ksEditor.

## ar12_gentry: AR-12 Bentonville to Gentry (point-to-point)

* Arkansas Highway 12 from SW Regional Airport Blvd in west Bentonville
  (36.33597, −94.24533) to 19353–19417 W Highway 12 near Gentry, AR, running past the
  north side of XNA airport and through Highfill.
* 28.1 km of road; 27.9 km from the start gate to the finish gate. 58 m of elevation change, max grade 7.9%.
* 8 m wide road with 3 m grass shoulders, a 150 m strip of real terrain on each side,
  and barriers 8 m from the road edge. Barrier walls close off both ends of the road.
* Red/white rumble kerbs on both sides of 17 corners, and 27 foam brake-marker boards
  (200/150/100/50 m) before the 8 tighter ones. The boards have no collision.
* 6 pit boxes and a 6-car grid behind the start gate (`AC_AB_START`), and the finish gate
  (`AC_AB_FINISH`) 100 m before the end of the road.
* Best in **Practice** or **Hotlap**. Like other point-to-point tracks in AC, lap-timing
  apps can misbehave at the finish.
* No AI line yet, so AI opponents won't drive. To add one: enable *Developer apps* in
  Content Manager (Settings → Assetto Corsa → System), drive the road once with the
  **AI** app recording, and save. That writes `ai/fast_lane.ai`.
* Visuals are basic: flat asphalt, grass, kerbs, marker boards and striped barriers. No buildings, trees,
  junctions or road signs.

Rebuild or tweak it with `python -m trackgen tracks/ar12_gentry/track.yaml`; see the main README.

Credits: road data © OpenStreetMap contributors (ODbL). Elevation is from AWS Terrain
Tiles (USGS 3DEP and others).
