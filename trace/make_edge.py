import argparse
from PIL import Image, ImageFilter, ImageOps

parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('image', type=argparse.FileType('rb'),
                    metavar='IMAGE.png', nargs='?',
                    help='image to approximate')

parser.add_argument('-o', '--output-file', type=str, metavar='FILENAME.svg',
                    default=None,
                    help='output file name')

opts = parser.parse_args()

if __name__ == '__main__':
    image = Image.open(opts.image)
    image = image.filter(ImageFilter.FIND_EDGES).convert('1').convert('L')
    for i in range(3):
        image = image.filter(ImageFilter.MaxFilter(3))
    image = ImageOps.invert(image)

    basename = os.path.basename(opts.image.name)
    opts.basename, _ = os.path.splitext(basename)
    if opts.output_file is None:
        opts.output_file = opts.basename + '_edge.png'
    
    image.save(opts.output_file)