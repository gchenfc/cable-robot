#!zsh

# This script resizes the apriltag images to a reasonable printable size, since in the github repo,
#   they exist as like, 9x9 pixels.
# Download the original sorce images from https://github.com/AprilRobotics/apriltag-imgs
# We will use tagStandard41h12 as recommended by https://github.com/AprilRobotics/apriltag and
#   because it has the best accuracy according to Figure 8 of
#   https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf
# Place the apriltag-imgs/tagStandard41h12 folder in this directory.

dpi=72
size_in="7.5"
size_px=$(echo "scale=1; $size_in * $dpi" | bc) # scale is the number of decimal points``
scale=$(echo "scale=0; $size_px * 100 / 9" | bc)
true_size=$(echo "scale=3; $scale * 9.0 / 100.0 / $dpi" | bc)
echo "scale = $scale %"
echo "true size = $true_size in"
echo "print at $dpi dpi"

for i in $(seq -f "%05g" 0 25); do
  convert tagStandard41h12/tag41_12_$i.png -scale $scale% tags/tagStandard41h12_$i.png
done