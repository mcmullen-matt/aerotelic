DUST case written by Aerotelic
==============================

Run these three, in this order, from this directory:

dust_pre  dust_pre.in
dust      dust.in
dust_post dust_post.in

  dust_pre   reads dust_pre.in and the components/ files, and
             collects the meshes in geometry/ into geo_input.h5.
  dust       reads dust.in and solves, writing one result file per
             time step into Output/.
  dust_post  reads dust_post.in and turns those into the VTK series
             and the load tables in Postprocessing/.

On Windows the executables are in the DUST install directory; put
it on PATH first. The three names are dust_pre.exe, dust.exe and
dust_post.exe.

This case
---------
  2 component(s), 2240 panels, 1124 nodes
  free stream 30 at 0 deg incidence
  320 steps of 0.00244707 s to t = 0.783063
  dense panel system about 38.28 MB

Results land in Output/ and Postprocessing/. Both are emptied by
nothing: delete them yourself between runs if you want a clean set.
