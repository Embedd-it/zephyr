.. _msc_test:

MSC Host Driver Test Sample
############################

Overview
********

The test waits a while for an USB drive to be connected and then tries to mount it
and do some file operations, verifying their outcome.

The test can be ran either on the ``native_sim`` target (in which case all communication
passes through the virtual USB stack) or the ``ek_ra8m2`` development kit.

If executed on the development kit one should connect a USB thumb drive (or similar
MSC device) to the USB HS interface. On the simulated target on the other hand
a virtual drive is created with three logical units.
