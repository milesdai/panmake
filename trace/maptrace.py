# -*- encoding: utf-8 -*-

from __future__ import print_function

import sys, re, os, argparse, heapq
from datetime import datetime
from collections import namedtuple, defaultdict
import numpy as np
from PIL import Image
from scipy import ndimage

######################################################################

DIR_RIGHT = 0
DIR_DOWN = 1
DIR_LEFT = 2
DIR_UP = 3

NEIGHBOR_OFFSET = np.array([
    [ 0, 1 ],
    [ 1, 0 ],
    [ 0, -1 ],
    [ -1, 0 ]
])

TURN_RIGHT = np.array([ DIR_DOWN, DIR_LEFT, DIR_UP, DIR_RIGHT ])
TURN_LEFT = np.array([ DIR_UP, DIR_RIGHT, DIR_DOWN, DIR_LEFT ])

VMAP_OFFSET = np.array([
    [ -1, 0, 0 ],
    [ 0, 0,  1 ],
    [ 0, 0,  0 ],
    [ 0, -1, 1 ]
])

DIAG_OFFSET = NEIGHBOR_OFFSET + NEIGHBOR_OFFSET[TURN_LEFT]
OPP_OFFSET = NEIGHBOR_OFFSET[TURN_LEFT]

CROSS_ELEMENT = np.array([[0,1,0],[1,1,1],[0,1,0]],dtype=np.bool)
BOX_ELEMENT = np.ones((3,3), dtype=np.bool)

######################################################################
# Some helper classes

EdgeInfo = namedtuple('EdgeInfo', ['node0', 'node1', 'label0', 'label1'])
EdgeRef = namedtuple('EdgeRef', ['edge_index', 'opp_label', 'step'])

######################################################################
# Class to store boundary representation for our map

class BoundaryRepresentation(object):

    def __init__(self):

        # list of nodes (points) or None for deleted 
        self.node_list = []

        # list of sets of edge indices
        self.node_edges = []

        # list of point arrays (or empty for deleted edges)
        self.edge_list = []

        # list of EdgeInfo (or None for deleted edges)
        self.edge_infolist = []

        # map from point to node index
        self.node_lookup = dict()
        
        # map from EdgeInfo to edge index
        self.edge_lookup = dict()

        # map from label to list of list of EdgeRef
        self.label_lookup = defaultdict(list)

        # a list of left-right convex boundaries
        self.convex_bounds = []

    def lookup_node(self, point, insert=False):

        key = tuple(map(float, point))

        if insert and key not in self.node_lookup:
            node_idx = len(self.node_list)
            self.node_list.append(point.copy())
            self.node_edges.append(set())
            self.node_lookup[key] = node_idx
        else:
            node_idx = self.node_lookup[key]

        return node_idx

    def add_edges(self, cur_label, contour_edges):

        edge_refs = []

        for opp_label, edge in contour_edges:

            assert cur_label != opp_label
            assert cur_label != 0

            label0 = min(cur_label, opp_label)
            label1 = max(cur_label, opp_label)

            if label0 == cur_label:
                step = 1
            else:
                step = -1

            edge_to_add = edge[::step]

            node0 = self.lookup_node(edge_to_add[0], insert=True)
            node1 = self.lookup_node(edge_to_add[-1], insert=True)

            edge_info = EdgeInfo(node0, node1, label0, label1)

            if edge_info in self.edge_lookup:

                edge_idx = self.edge_lookup[edge_info]

                stored_edge = self.edge_list[edge_idx]

                assert self.edge_infolist[edge_idx] == edge_info
                assert np.all(stored_edge == edge_to_add)
                assert edge_idx in self.node_edges[node0]
                assert edge_idx in self.node_edges[node1]

            else:

                edge_idx = len(self.edge_list)
                self.edge_list.append( edge_to_add )
                self.edge_infolist.append( edge_info )

                self.edge_lookup[edge_info] = edge_idx

                self.node_edges[node0].add( edge_idx )
                self.node_edges[node1].add( edge_idx )

            edge_refs.append(EdgeRef(edge_idx, opp_label, step))
        
        self.label_lookup[cur_label].append( edge_refs)

    def replace_endpoints(self, edge_idx, na, nb, nc):

        edge = self.edge_list[edge_idx]
        edge_info = self.edge_infolist[edge_idx]

        assert (edge_info.node0 == na or edge_info.node0 == nb or
                edge_info.node1 == na or edge_info.node1 == nb)

        n0 = None
        n1 = None

        if edge_info.node0 == na:
            n0 = na
            new_n0 = nc
        elif edge_info.node0 == nb:
            n0 = nb
            new_n0 = nc
        else:
            new_n0 = edge_info.node0

        if edge_info.node1 == na:
            n1 = na
            new_n1 = nc
        elif edge_info.node1 == nb:
            n1 = nb
            new_n1 = nc
        else:
            new_n1 = edge_info.node1

        if n0 is not None and n1 is not None:
            self.edge_list[edge_idx] = edge[:0]
            self.edge_infolist[edge_idx] = None
            # NB we will rebuild label_lookup after all merges
            return

        self.node_edges[nc].add(edge_idx)

        pc = self.node_list[nc]


        for node_idx, which_end, lo, hi in [(n0, 0, 1, 0), (n1, -1, 0, 1)]:
            if node_idx is None:
                continue
            p = self.node_list[node_idx]
            delta = (pc - p).reshape(1, 2)
            u = np.linspace(lo, hi, len(edge)).reshape(-1, 1)
            edge = edge + delta * u
            edge[which_end] = pc


        edge_info = EdgeInfo(new_n0, new_n1, edge_info.label0, edge_info.label1)

        self.edge_list[edge_idx] = edge
        self.edge_infolist[edge_idx] = edge_info
        
        assert np.all(edge[0] == self.node_list[edge_info.node0])
        assert np.all(edge[-1] == self.node_list[edge_info.node1])
            
    def merge_nodes(self, tol):

        node_points = np.array(self.node_list)

        rng = range(len(node_points))

        i, j = np.meshgrid(rng, rng)
        use = i > j
        
        i = i[use]
        j = j[use]

        ni = node_points[i]
        nj = node_points[j]

        dists = np.linalg.norm(ni - nj, axis=1)

        heap = list(zip(dists, i, j))
        heapq.heapify(heap)
        
        retired_nodes = set()
        active_nodes = set(rng)

        while len(heap):

            dmin, na, nb = heapq.heappop(heap)
            assert na > nb

            if dmin > tol:
                break
            
            if na in retired_nodes or nb in retired_nodes:
                continue
            
            print('  merge nodes {} and {} with distance {}'.format(
                na, nb, dmin))

            pa = self.node_list[na]
            pb = self.node_list[nb]
            
            pc = 0.5*(pa + pb)

            nc = len(self.node_list)
            nkey = tuple(map(float, pc))
            
            self.node_list.append(pc.copy())
            self.node_edges.append(set())
            self.node_lookup[nkey] = nc

            assert self.lookup_node(pc) == nc
            
            for node_idx in (na, nb):
                for edge_idx in self.node_edges[node_idx]:
                    if self.edge_infolist[edge_idx] is not None:
                        self.replace_endpoints(edge_idx, na, nb, nc)

            for node_idx in (na, nb):
                p = self.node_list[node_idx]
                pkey = tuple(map(float, p))
                del self.node_lookup[pkey]
                self.node_list[node_idx] = None
                self.node_edges[node_idx] = set()
                retired_nodes.add(node_idx)
                active_nodes.remove(node_idx)

            for nj in active_nodes:
                pj = self.node_list[nj]
                dcj = np.linalg.norm(pc - pj)
                hkey = (dcj, nc, nj)
                heapq.heappush(heap, hkey)

            active_nodes.add(nc)

        # rebuild label lookup
        new_label_lookup = dict()
        
        for label, contours in self.label_lookup.items():

            new_contours = []
            
            for contour in contours:
                
                new_contour = []
                
                for edge_ref in contour:
                    idx, _, _ = edge_ref
                    if self.edge_infolist[idx] is not None:
                        new_contour.append(edge_ref)

                if len(new_contour):
                    new_contours.append(new_contour)

            if len(new_contours):
                new_label_lookup[label] = new_contours
            else:
                print('totally deleted label {}!'.format(label))
            
        self.label_lookup = new_label_lookup

    def center_image(self, opts):
        new_center = (opts.center, opts.center)
        new_scale = opts.scale
        # given the image, center such that everything is within the
        # bounding box of (100, 100) to (900, 900)
        min_X_coord = min([edgeset.min(axis=0)[0] for edgeset in self.edge_list if edgeset.any()])
        min_Y_coord = min([edgeset.min(axis=0)[1] for edgeset in self.edge_list if edgeset.any()])
        max_X_coord = max([edgeset.max(axis=0)[0] for edgeset in self.edge_list if edgeset.any()])
        max_Y_coord = max([edgeset.max(axis=0)[1] for edgeset in self.edge_list if edgeset.any()])

        center_X, center_Y = (min_X_coord + max_X_coord) / 2., (min_Y_coord + max_Y_coord) / 2.

        center = np.array([[center_X, center_Y]])
        new_center = np.array(new_center).reshape(1, 2)
        print("Center: ", center)
        scale = new_scale / max(max_X_coord - min_X_coord, max_Y_coord - min_Y_coord)
        print("Scale: ", scale)

        for i in range(len(self.edge_list)):
            if self.edge_list[i].any():
                self.edge_list[i] = (self.edge_list[i] - center) * scale + new_center
        
        for i in range(len(self.node_list)):
            if self.node_list[i] is not None:
                self.node_list[i] = (self.node_list[i] - center) * scale + new_center

    def find_convex_boundary(self, opts):
        image_height = opts.image_size
        grid_height = opts.grid_size
        for k, y in enumerate(range(0, image_height, grid_height)):
            min_intersect = 1e9; max_intersect = -1e9
            for i in range(len(self.edge_list) - 1):
                edgeset = self.edge_list[i+1]
                if ((opts.center - opts.scale // 2) in edgeset) or ((opts.center + opts.scale // 2) in edgeset):
                    continue
                if not edgeset.any():
                    continue
                for j, _ in enumerate(edgeset):
                    first, second = edgeset[j], edgeset[(j+1) % len(edgeset)]
                    if (first[1] - y) * (second[1] - y) > 0:
                        # this segment does not intersect
                        pass
                    elif (first[1] - y) * (second[1] - y) == 0:
                        # either of the points are on the line y
                        if (first[1] - y) == 0:
                            min_intersect = min(first[0], min_intersect)
                            max_intersect = max(first[0], max_intersect)
                        if (second[1] - y) == 0:
                            min_intersect = min(second[0], min_intersect)
                            max_intersect = max(second[0], max_intersect)
                    else: # it clearly intersects
                        # assume t * b1 + (1-t) * b2 = y
                        t = (y - second[1]) / (first[1] - second[1])
                        intersection_point = first[0] * t + second[0] * (1-t)
                        min_intersect = min(intersection_point, min_intersect)
                        max_intersect = max(intersection_point, max_intersect)

            if min_intersect <= image_height and max_intersect >= 0:
                # assume this image is connected in the Y direction.
                if (len(self.convex_bounds) // 2) % 2 == 0:
                    self.convex_bounds += [[min_intersect + opts.grid_size // 2, y], 
                                           [max_intersect - opts.grid_size // 2, y]]
                else: # put every odd point in the opposite direction
                    self.convex_bounds += [[max_intersect - opts.grid_size // 2, y], 
                                           [min_intersect + opts.grid_size // 2, y]]
                
        self.convex_bounds = np.array(self.convex_bounds, dtype='uint8')

    def save_debug_image(self, opts, orig_shape, colors, name):
        
        filename = opts.basename + '_debug_' + name + '.svg'
        
        with open(filename, 'w') as svg:
            
            svg.write('<svg width="{}" height="{}" '
                      'xmlns="http://www.w3.org/2000/svg">\n'.
                      format(orig_shape[1], orig_shape[0]))

            svg.write(' <rect width="100%" height="100%" fill="#eee" />\n')

            for ilabel in range(2):

                if ilabel == 0:
                    svg.write(' <g stroke-linejoin="miter" stroke-width="4" fill="none">\n')
                else:
                    svg.write(' <g stroke-linejoin="miter" stroke-width="4" fill="none" stroke-dasharray="8, 8" >\n')

                for edge, einfo in zip(self.edge_list[1:], self.edge_infolist[1:]):
                    svg.write('  <path d="')
                    last = np.array([0,0])
                    for i, pt in enumerate(edge):
                        pt = pt.astype(int)
                        if i == 0:
                            svg.write('M{},{}'.format(pt[0], pt[1]))
                        else:
                            diff = pt - last
                            if diff[1] == 0:
                                svg.write('h{}'.format(diff[0]))
                            elif diff[0] == 0:
                                svg.write('v{}'.format(diff[1]))
                            else:
                                svg.write('l{},{}'.format(*diff))
                        last = pt

                    color = colors[einfo.label0 if ilabel == 0 else einfo.label1]
                    svg.write('" stroke="#{:02x}{:02x}{:02x}" />\n'.format(*color))

                svg.write(' </g>\n')

            svg.write(' <g stroke="none" fill="#000">\n')
            for pt in self.node_list:
                svg.write('  <circle cx="{}" cy="{}" r="4" />\n'.format(*pt))
            svg.write(' </g>\n')

                

                
            svg.write('</svg>\n')

        print('wrote', filename)
        
######################################################################
# Input is string, output is pair (string, lambda image -> image)

def filter_type(fstr):
    
    m = re.match(r'^\s*([a-z]+)\s*:\s*([a-z]+)\s*,\s*([1-9][0-9]*)\s*$', fstr)

    if m is None:
        raise argparse.ArgumentTypeError('invalid filter string')
        
    operation = m.group(1)
    element = m.group(2)
    iterations = int(m.group(3))

    fnmap = dict(
        open=ndimage.binary_opening,
        close=ndimage.binary_closing,
        dilate=ndimage.binary_dilation,
        erode=ndimage.binary_erosion)

    if operation not in fnmap.keys():
        raise argparse.ArgumentTypeError('invalid operation ' + operation)

    if element == 'box':
        element = BOX_ELEMENT
    elif element == 'cross':
        element = CROSS_ELEMENT
    else:
        raise argparse.ArgumentTypeError('invalid element ' + element)

    f = lambda img: fnmap[operation](img, element, iterations=iterations)
        
    return fstr, f

######################################################################
# Confirm with [y/n]

def confirm(prompt):

    while True:
        
        print(prompt + ' [y/n]: ', end='')
        sys.stdout.flush()

        choice = input().lower()

        if choice in ['y', 'yes']:
            return True
        elif choice in ['n', 'no']:
            return False
        else:
            print('invalid choice')

######################################################################
# Parse command-line options, return namespace containing results

def get_options():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('image', type=argparse.FileType('rb'),
                        metavar='IMAGE.png', nargs='?',
                        help='image to approximate')

    parser.add_argument('-z', '--zoom', type=float, metavar='ZOOM',
                        default=1.0,
                        help='amount to resize image on load')

    parser.add_argument('-t', '--threshold', type=int, metavar='T',
                        default=64,
                        help='intensity threshold for outlines')

    parser.add_argument('-a', '--alpha-threshold', type=int, metavar='T',
                        default=127,
                        help='threshold for alpha channel')

    parser.add_argument('-C', '--connectivity', choices=('4','8'),
                        default='4',
                        help='connectivity of non-outline regions')

    parser.add_argument('-f', '--filter', type=filter_type, default=None,
                        help='filter for preprocessing outline map '
                        'after thresholding but before connected '
                        'component analysis; must be of the format '
                        '(erode|dilate|open|close):(box|cross),ITERATIONS '
                        'e.g., erode:cross,1')

    parser.add_argument('-e', '--edge-tol', type=float, metavar='E',
                        default='1.42',
                        help='tolerance in px for simplifying edges')

    parser.add_argument('-n', '--node-tol', type=float, metavar='N',
                        default=0,
                        help='tolerance in px for merging nodes')
    
    parser.add_argument('-o', '--output-file', type=str, metavar='FILENAME.svg',
                        default=None,
                        help='output SVG file name')

    parser.add_argument('-s', '--stroke-width', type=float, metavar='S',
                        default=1.0,
                        help='output SVG stroke width')

    parser.add_argument('-b', '--bg-stroke-width', type=float, metavar='S',
                        default=None,
                        help='output SVG stroke width for largest region')
    
    parser.add_argument('-d', '--debug-images', action='store_true',
                        help='generate debug images')

    parser.add_argument('-D', '--allow-dark-colors', action='store_true',
                        help='flag to prevent applying grayscale threshold '
                        'to image supplied with -c')
    
    parser.add_argument('-m', '--min-area', type=int, metavar='A',
                        default=1, help='minimum region area in pixels')

    parser.add_argument('-c', '--color-image', type=argparse.FileType('rb'),
                        default=None, help='image to supply color for output map')

    parser.add_argument('-q', '--color-quantize-bits', type=int,
                        default=8, help='quantization for finding region '
                        'colors with -c')

    parser.add_argument('-r', '--random-colors', action='store_true',
                        help='color regions randomly')

    parser.add_argument('-R', '--random-seed', type=int,
                        help='random seed for colors')

    parser.add_argument('-y', '--overwrite', action='store_true',
                        help='overwrite output')

    parser.add_argument('-g', '--grid-size', action='store_true',
                        default=20, help='Set grid size of the pancake batter')

    parser.add_argument('-I', '--image-size', action='store_true',
                        default=256, help='Set image size after rescaling')

    parser.add_argument('-x', '--center', action='store_true',
                        default=127, help='Set image center after rescaling')

    parser.add_argument('-O', '--scale', action='store_true',
                        default=240, help='Set image scale after rescaling')
    
    opts = parser.parse_args()

    if opts.image is None:
        if opts.color_image is None:
            print('error: must provide image filename or set color image with -c')
            sys.exit(1)
        else:
            opts.image = open(opts.color_image.name, 'rb')

    basename = os.path.basename(opts.image.name)
    opts.basename, _ = os.path.splitext(basename)

    if opts.bg_stroke_width is None:
        opts.bg_stroke_width = opts.stroke_width

    if opts.output_file is None:
        opts.output_file = opts.basename + '.svg'

    if os.path.exists(opts.output_file) and not opts.overwrite:
        if not confirm(opts.output_file + ' exists. Overwrite?'):
            print('will not overwite output, exiting')
            sys.exit(1)

    return opts
    
######################################################################
# Downsample pixel values, rounding to center of bins.

def quantize(image, bits_per_channel=None):

    if bits_per_channel is None:
        bits_per_channel = 8

    assert image.dtype == np.uint8

    shift = 8-bits_per_channel
    halfbin = (1 << shift) >> 1

    return ((image.astype(int) >> shift) << shift) + halfbin

######################################################################
# Pack RGB triplets into ints

def pack_rgb(rgb):

    orig_shape = None

    if isinstance(rgb, np.ndarray):
        assert rgb.shape[-1] == 3
        orig_shape = rgb.shape[:-1]
    else:
        assert len(rgb) == 3
        rgb = np.array(rgb)

    rgb = rgb.astype(int).reshape((-1, 3))

    packed = (rgb[:, 0] << 16 |
              rgb[:, 1] << 8 |
              rgb[:, 2])

    if orig_shape is None:
        return packed
    else:
        return packed.reshape(orig_shape)

######################################################################
# Unpack ints to RGB triplets

def unpack_rgb(packed):

    orig_shape = None

    if isinstance(packed, np.ndarray):
        assert packed.dtype == int
        orig_shape = packed.shape
        packed = packed.reshape((-1, 1))

    rgb = ((packed >> 16) & 0xff,
           (packed >> 8) & 0xff,
           (packed) & 0xff)

    if orig_shape is None:
        return rgb
    else:
        return np.hstack(rgb).reshape(orig_shape + (3,))

######################################################################
# Get the dominant color in a list of colors (with optional
# quantization)

def get_dominant_color(colors, bits_per_channel=None):

    assert colors.shape[-1] == 3

    quantized = quantize(colors, bits_per_channel).astype(int)
    packed = pack_rgb(quantized)

    unique, counts = np.unique(packed, return_counts=True)

    packed_mode = unique[counts.argmax()]

    return unpack_rgb(packed_mode)

######################################################################
# Save a debug image if allowed

def save_debug_image(opts, name, image):

    if not opts.debug_images:
        return

    if isinstance(image, np.ndarray):

        if image.dtype == np.bool:
            image = (image.astype(np.uint8) * 255)

        if len(image.shape) == 2:
            mode = 'L'
        else:
            mode = 'RGB'

        image = Image.fromarray(image, mode)

    filename = opts.basename + '_debug_' + name + '.png'
    image.save(filename)

    print('wrote', filename)

######################################################################
# Open an input image and get the RGB colors as well as the mask

def get_mask(input_image, opts):
    
    rgb = input_image
    alpha = None

    if (rgb.mode == 'LA' or
        (rgb.mode == 'P' and 'transparency' in rgb.info)):
        rgb = rgb.convert('RGBA')

    if rgb.mode == 'RGBA':
        alpha = np.array(rgb.split()[-1])
        
    rgb = rgb.convert('RGB')
    rgb = np.array(rgb)
        
    gray = rgb.max(axis=2)

    mask = (gray > opts.threshold)
    
    if alpha is not None:
        mask = mask | (alpha < opts.alpha_threshold)

    save_debug_image(opts, 'mask', mask)
        
    if opts.filter is not None:
        print('applying filter:', opts.filter[0])
        mask = opts.filter[1](mask)
        save_debug_image(opts, 'mask_filtered', mask)

    return mask

######################################################################

def printp(*args):
    print(*args, end='')
    sys.stdout.flush()
    
######################################################################

def get_labels_and_colors(mask, opts):

    if opts.connectivity == '8':
        structure = BOX_ELEMENT
    else:
        structure = CROSS_ELEMENT

    labels, num_labels = ndimage.label(mask, structure=structure)
    print('found {} labels'.format(num_labels))

    unlabeled = ~mask
    

    printp('computing areas... ')
    start = datetime.now()

    areas, bins = np.histogram(labels.flatten(),
                               bins=num_labels,
                               range=(1, num_labels+1))

    elapsed = (datetime.now() - start).total_seconds()

    print('finished computing areas in {} seconds.'.format(elapsed))

    idx = np.hstack( ([0], np.argsort(-areas)+1) )

    replace = np.zeros_like(idx)
    replace[idx] = range(len(idx))

    labels = replace[labels]
    areas = areas[idx[1:]-1]

    print('min area is {}, max is {}'.format(areas[-1], areas[0]))
    
    if opts.min_area > areas[-1]:
        print('killing all labels with area < {} px'.format(opts.min_area))
        kill_labels = np.nonzero(areas < opts.min_area)[0]
        num_labels = kill_labels.min()
        kill_mask = (labels > num_labels)
        save_debug_image(opts, 'kill_labels', kill_mask)
        unlabeled = unlabeled | kill_mask
        print('killed {} labels, now at {} total'.format(
            len(kill_labels), num_labels))

    colors = 255*np.ones((num_labels+1,3), dtype=np.uint8)
    
    if opts.color_image is not None:
        color_image = Image.open(opts.color_image)
        labels_size = labels.shape[::-1]
        if color_image.size != labels_size:
            color_image = color_image.resize(labels_size, Image.NEAREST)
        color_image = np.array(color_image.convert('RGB'))
        print('assigning colors from {}...'.format(opts.color_image.name))
        slices = ndimage.find_objects(labels, num_labels)
        for label, (yslc, xslc) in zip(range(1, num_labels+1), slices):
            print('  coloring label {}/{}'.format(label, num_labels))
            lmask = (labels[yslc,xslc] == label)
            crect = color_image[yslc,xslc]
            if not opts.allow_dark_colors:
                lmask = lmask & (crect.max(axis=2) > opts.threshold)
            if not np.any(lmask):
                print('no colors available for label {}, '
                      'try running with -D?'.format(label))
            else:
                colors[label] = get_dominant_color(crect[lmask],
                                                   opts.color_quantize_bits)
    elif opts.random_colors:
        
        if opts.random_seed is not None:
            np.random.seed(opts.random_seed)
            
        colors = np.random.randint(128, size=(num_labels+1,3),
                                   dtype=np.uint8) + 128
        colors[0,:] = 255

    save_debug_image(opts, 'regions', colors[labels])
        
    printp('running DT... ')
    
    start = datetime.now()


    result = ndimage.distance_transform_edt(unlabeled,
                                            return_distances=opts.debug_images,
                                            return_indices=True)

    if opts.debug_images:
        dist, idx = result
        dist /= dist.max()
        dist = (dist*255).astype(np.uint8)
        save_debug_image(opts, 'dist', dist)
    else:
        idx = result
    
    elapsed = (datetime.now() - start).total_seconds()
    print('ran DT in {} seconds'.format(elapsed))
    
    labels = labels[tuple(idx)]
    assert not np.any(labels == 0)

    labels_big = np.zeros((labels.shape[0]+2,labels.shape[1]+2),
                          dtype=labels.dtype)

    labels_big[1:-1,1:-1] = labels

    start = datetime.now()
    printp('finding objects... ')
    slices = ndimage.find_objects(labels, num_labels)
    elapsed = (datetime.now() - start).total_seconds()
    print('found all objects in {} seconds'.format(elapsed))

    slices_big = []

    for spair in slices:
        spair_big = []
        for s, dmax in zip(spair, labels.shape):
            spair_big.append(slice(s.start, s.stop+2))
        slices_big.append( tuple(spair_big) )

    assert labels_big.min() == 0 and labels_big.max() == num_labels
    assert len(slices) == num_labels

    save_debug_image(opts, 'regions_expanded', colors[labels_big[1:-1, 1:-1]])
    
    return num_labels, labels_big, slices_big, colors
    
######################################################################

def follow_contour(l_subrect, cur_label,
                   startpoints, pos):

    start = pos
    cur_dir = DIR_RIGHT
    
    contour_info = []

    while True:

        ooffs = OPP_OFFSET[cur_dir]
        noffs = NEIGHBOR_OFFSET[cur_dir]
        doffs = DIAG_OFFSET[cur_dir]

        neighbor = tuple(pos + noffs)
        diag = tuple(pos + doffs)
        opp = tuple(pos + ooffs )

        assert l_subrect[pos] == cur_label
        assert l_subrect[opp] != cur_label

        contour_info.append( pos + (cur_dir, l_subrect[opp]) )

        startpoints[pos] = False

        if l_subrect[neighbor] != cur_label:
            cur_dir = TURN_RIGHT[cur_dir]
        elif l_subrect[diag] == cur_label:
            pos = diag
            cur_dir = TURN_LEFT[cur_dir]
        else:
            pos = neighbor

        if pos == start and cur_dir == DIR_RIGHT:
            break

    n = len(contour_info)
    contour_info = np.array(contour_info)
    
    clabels = contour_info[:,3]

    # set of unique labels for this contour
    opp_label_set = set(clabels)
    assert cur_label not in opp_label_set

    # if multiple labels and one wraps around, correct this
    if len(opp_label_set) > 1 and clabels[0] == clabels[-1]:
        idx = np.nonzero(clabels != clabels[0])[0][0]
        perm = np.hstack( (np.arange(idx, n),
                           np.arange(idx)) )
        contour_info = contour_info[perm]
        clabels = contour_info[:,3]

    # make sure no wraparound
    assert len(opp_label_set) == 1 or clabels[0] != clabels[-1]
    
    # apply offset to get contour points
    cpoints = contour_info[:,:2].astype(np.float32)
    cdirs = contour_info[:,2]
    cpoints += 0.5 * (OPP_OFFSET[cdirs] - NEIGHBOR_OFFSET[cdirs] + 1)

    # put points in xy format
    cpoints = cpoints[:,::-1]

    if len(opp_label_set) == 1:
        idx = np.arange(len(cpoints))
        xyi = zip(cpoints[:,0], cpoints[:,1], idx)
        imin = min(xyi)
        i = imin[-1]
        cpoints = np.vstack( ( cpoints[i:], cpoints[:i] ) )
        assert np.all(clabels == clabels[0])

    return cpoints, clabels

######################################################################

def split_contour(cpoints, clabels):
    
    edges = []

    shifted = np.hstack(( [-1], clabels[:-1] ))
    istart = np.nonzero( clabels - shifted )[0]
    iend = np.hstack( (istart[1:], len(clabels)) )

    for start, end in zip(istart, iend):
        assert start == 0 or clabels[start] != clabels[start-1]
        assert clabels[end-1] == clabels[start]
        opp_label = clabels[start]
        if end < len(cpoints):
            edge = cpoints[start:end+1]
        else:
            edge = np.vstack( (cpoints[start:end], cpoints[0]) )
        edges.append( (opp_label, edge) )
        start = end

    return edges

######################################################################

def store_contour_edges(opts, labels,
                        edge_lookup, edge_list,
                        cur_label, contour_edges):

    edge_refs = []

    for opp_label, edge in contour_edges:

        assert cur_label != opp_label
        assert cur_label != 0

        print('  storing contour edge with cur={}, opp={}'.format(
            cur_label, opp_label))

        lmin = min(cur_label, opp_label)
        lmax = max(cur_label, opp_label)

        if lmin == cur_label:
            step = 1
        else:
            step = -1

        edge_to_add = edge[::step]
        p0 = tuple(map(float, edge_to_add[0]))
        p1 = tuple(map(float, edge_to_add[1]))

        key = (lmin, lmax, p0, p1)

        if key in edge_lookup:
            idx = edge_lookup[key]
            if not np.all(edge_list[idx] == edge_to_add):
                debug = 255*np.ones(labels.shape + (3,), dtype=np.uint8)
                debug[labels == cur_label] = (255, 0, 0)
                debug[labels == opp_label] = (0, 0, 255)
                save_debug_image(opts, 'debug_edge', debug)
                print('not forward/backward symmetric!')
                print(type(edge_to_add))
                print(type(edge_list[idx]))
                print(edge_list[idx].shape, edge_list[idx].dtype)
                print(edge_to_add.shape, edge_to_add.dtype)
                print(edge_to_add == edge_list[idx])
                assert np.all(edge_list[idx] == edge_to_add)
        else:
            idx = len(edge_list)
            edge_list.append( edge_to_add )
            edge_lookup[key] = idx

        edge_refs.append( (idx, opp_label, step) )

    return edge_refs

######################################################################

def _simplify_r(opts, p0, edge, output_list):

    assert np.all( output_list[-1][-1] == p0 )
    assert not np.all(edge[0] == p0)
    
    p1 = edge[-1]
    
    if len(edge) == 1:
        output_list.append(edge)
        return

    l = np.cross([p0[0], p0[1], 1], [p1[0], p1[1], 1])
    n = l[:2]
    w = np.linalg.norm(n)

    if w == 0:
        dist = np.linalg.norm(edge - p0, axis=1)
        idx = dist.argmax()
        dmax = np.inf
    else:
        l /= w
        dist = np.abs( np.dot(edge, l[:2]) + l[2] )
        idx = dist.argmax()
        dmax = dist[idx]

    if dmax < opts.edge_tol:
        output_list.append(np.array([p1]))
    elif len(edge) > 3:
        _simplify_r(opts, p0, edge[:idx+1], output_list)
        _simplify_r(opts, edge[idx], edge[idx+1:], output_list)
    else:
        output_list.append(edge)

######################################################################
    
def simplify(opts, edge):

    if not len(edge):
        return edge

    p0 = edge[0]
    output_list = [ edge[[0]] ]

    _simplify_r(opts, p0, edge[1:], output_list)

    return np.vstack( tuple(output_list) )

######################################################################

def build_brep(opts, num_labels, labels, slices, colors):

    brep = BoundaryRepresentation()

    label_range = range(1, num_labels+1)

    print('building boundary representation...')
    
    # for each object
    for cur_label, (yslc, xslc) in zip(label_range, slices):

        p0 = (xslc.start-1, yslc.start-1)

        # extract sub-rectangle for this label
        l_subrect = labels[yslc, xslc]

        # get binary map of potential start points for contour in
        # rightward direction
        mask_subrect = (l_subrect == cur_label)

        mask_shifted_down = np.vstack(
            (np.zeros_like(mask_subrect[0].reshape(1,-1)),
             mask_subrect[:-1]))

        startpoints = mask_subrect & ~mask_shifted_down
        
        print('  processing label {}/{} with area {}'.format(
            cur_label, num_labels, (l_subrect == cur_label).sum()))

        # while there are candidate locations to start at
        while np.any(startpoints):

            # get the first one
            i, j = np.nonzero(startpoints)
            pos = (i[0], j[0])

            # extract points and adjacent labels along contour,
            # this modifies startpoints
            cpoints, clabels = follow_contour(l_subrect, cur_label,
                                              startpoints, pos)

            cpoints += p0

            # split contour into (opp_label, points) pairs
            contour_edges = split_contour(cpoints, clabels)

            # add them to our boundary representation
            brep.add_edges(cur_label, contour_edges)

    if opts.debug_images:
        orig_shape = (labels.shape[0]-2, labels.shape[1]-2)
        brep.save_debug_image(opts, orig_shape, colors, 'brep')

    simplified = False

    if opts.node_tol > 0:
        print('merging all nodes closer than {} px...'.format(opts.node_tol))
        brep.merge_nodes(opts.node_tol)
        simplified = True

    brep.center_image(opts)

    if opts.edge_tol > 0:
        print('simplifying edges...')
        brep.edge_list = [ simplify(opts, edge) for edge in brep.edge_list ]
        simplified = True

    print (brep.edge_list)

    print ("Convex bounds:")
    brep.find_convex_boundary(opts)
    print (brep.convex_bounds)

    if opts.debug_images and simplified:
        orig_shape = (labels.shape[0]-2, labels.shape[1]-2)
        brep.save_debug_image(opts, orig_shape, colors, 'brep_simplified')
        
    return brep

######################################################################

def num_fmt(n):
    s = '{:.2f}'.format(n)
    if '.' in s:
        s = re.sub(r'\.?0+$', '', s)
    return s

def output_svg(opts, orig_shape, brep, colors):
    processed = ''
    with open(opts.output_file, 'w') as svg:

        svg.write('<svg width="{}" height="{}" '
                  'xmlns="http://www.w3.org/2000/svg">\n'.
                  format(orig_shape[1], orig_shape[0]))

        svg.write(' <g stroke="#000" stroke-linejoin="bevel" '
                  'stroke-width="{}">\n'.format(opts.stroke_width))
            
        cpacked = pack_rgb(colors.astype(int))
        cset = set(cpacked)

        lsets = []

        for c in cset:
            idx = np.nonzero(cpacked == c)[0]
            if 1 in idx:
                lsets.insert(0, idx)
            else:
                lsets.append(idx)

        assert 1 in lsets[0]

        for lset in lsets:
            # svg.write('  <g fill="#{:02x}{:02x}{:02x}">\n'.format(
            #     *colors[lset[0]]))
            svg.write('  <g fill="#fff">\n')
            

            for cur_label in lset:

                if cur_label not in brep.label_lookup:
                    continue

                contours = brep.label_lookup[cur_label]
                svg.write('   <path d="')

                for i, contour in enumerate(contours):
                    first_command = ''
                    for j, (edge_idx, _, step) in enumerate(contour):

                        edge = brep.edge_list[edge_idx][::step].astype('uint8')
                        if ((opts.center - opts.scale // 2) in edge) or ((opts.center + opts.scale // 2) in edge):
                            break
                    
                        if j == 0:
                            svg.write('M{},{}'.format(*map(num_fmt, edge[0])))
                            first_command = 'm {} {}\n'.format(*map(num_fmt, edge[0]))
                            processed += first_command
                        for pt in edge[1:]:
                            svg.write('L{},{}'.format(*map(num_fmt, pt)))
                            processed += 'd {} {}\n'.format(*map(num_fmt, pt))
                    svg.write('Z')
                    processed += first_command

                svg.write('"')

                if cur_label == 1 and opts.stroke_width != opts.bg_stroke_width:
                    svg.write(' stroke-width="{}"'.format(opts.bg_stroke_width))

                svg.write('/>\n')

            svg.write('  </g>\n')
            
            processed += 'w\n'

            svg.write('<g fill="#ddd" fill-opacity="0.0">')
            svg.write('<path stroke="#ccc" ' +
                      'stroke-opacity="0.3" ' +
                      'stroke-linecap="round" ' +
                      'stroke-width="{0}" d="M{1:.2f},{2:.2f} '.format(opts.grid_size, brep.convex_bounds[0][0], brep.convex_bounds[0][1]))
            processed += 'm {} {}\n'.format(opts.grid_size, brep.convex_bounds[0][0], brep.convex_bounds[0][1])
            for point in brep.convex_bounds[1:]:
                svg.write('L{0:.2f},{1:.2f}'.format(point[0], point[1]))
                processed += 'd {} {}\n'.format(point[0], point[1])
            svg.write(' "/>')
            svg.write(' </g>')
        svg.write(' </g>\n')
        svg.write('</svg>\n')

    with open(opts.output_file[:-len('svg')] + 'txt', 'w') as f:
        f.write(processed)
    print('wrote', opts.output_file)
    
######################################################################

def main():

    opts = get_options()

    input_image = Image.open(opts.image)
    if opts.zoom != 1:
        w, h = input_image.size
        wnew = int(round(w*opts.zoom))
        hnew = int(round(h*opts.zoom))
        resample = Image.LANCZOS if opts.zoom > 1 else Image.LANCZOS
        input_image = input_image.resize((wnew, hnew), resample)
        save_debug_image(opts, 'resized', input_image)
    
    mask = get_mask(input_image, opts)

    # labels is a 2D array that ranges from 0 (background) to
    # num_labels (inclusive), and slices are bounding rectangles for
    # each non-zero label.
    num_labels, labels, slices, colors = get_labels_and_colors(mask, opts)

    brep = build_brep(opts, num_labels, labels, slices, colors)

    output_svg(opts, # mask.shape
               [1000, 1000], brep, colors)
        
######################################################################

if __name__ == '__main__':
    main()

