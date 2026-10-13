# Segmented top centre and CAD hole centre

SegmentedTopCenter consumes an externally supplied top mask and current RGB-D.
It does not segment automatically or command a robot. Range-valid even pixels
in an eroded mask fit the plane. All range-valid odd pixels in the original
mask validate it, without residual filtering. Contour rays intersect the
plane; a 3-D rectangle provides the geometric centre. CAD offsets are rotated.

estimate(frame, mask, qualified=...) separates candidate_available from
geometry_consistent and absolute_accuracy. Nominal size, coverage and plane
support are consistency checks, not absolute-error bounds. Pose validity and
motion_usable remain false.

Known one-end occlusion uses complete_boundary=False and visible_long_edge
positive/negative along the canonical long-axis sign convention. At least
half the long extent and both short bounds must remain. Missing length is CAD
completed and explicitly labelled. Unknown occlusion is not completed.
Automatic prompting, general occlusion detection and insertion-side selection
are outside this module. The current real-time pipeline is not switched.

Real evaluation uses twelve prior samples and first-frame manually prompted
SAM masks. Slight camera motion and mask reuse limit stability interpretation.
