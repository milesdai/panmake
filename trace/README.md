# maptrace

Usage (to create a trace preview in SVG and trace gcode file):

    python maptrace.py -c images/heart.png -f open:box,1 -b0 -y -e5

To create edges from images:

    python make_edge.py images/santa-bad.png

Cloned from https://github.com/mzucker/maptrace.
Produce watertight polygonal vector maps by tracing raster images.

## Requirements

 - Python 2 or 3
 - Numpy 1.10 or higher
 - Scipy 0.17 or higher
 - Pillow

## Examples

The images in the [`examples`](/examples) directory were created by running the following commands:

    python maptrace.py -c images/*.png -f open:box,1 -b0 -y -e5

## Software license

The `maptrace.py` program itself is provided under the [MIT license](/LICENSE).

