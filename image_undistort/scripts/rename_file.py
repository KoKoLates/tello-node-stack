from PIL import Image
import glob

seq = '05'
data_path = '/home/jy/Desktop/tello_aigc/' + 'seq{}/'.format(seq)
extension = '*.png'
output_path = '/home/jy/Desktop/tello_aigc_2/' + 'seq{}/'.format(seq)

files = glob.glob(data_path + extension)
files.sort()
num_image = len(files)

for n, file_path in enumerate(files):
    file_name = '{:06d}.png'.format(n)
    im = Image.open(file_path)
    im = im.save(output_path + file_name)
    print(file_name)
