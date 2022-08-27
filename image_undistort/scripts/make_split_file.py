import glob
seq = '06'
f = open("seq{}_split.txt".format(seq),"w+")
data_path = '/home/jy/Desktop/tello_aigc/' + 'seq{}/'.format(seq)
# data_path = '/home/usrg/Desktop/test/'
extension = '*.png'
path_in_split = 'seq{}/'.format(seq)
files = glob.glob(data_path + extension)
files.sort()
num_image = len(files)

for n, file_path in  enumerate(files):
    file_name = file_path.split('/')[-1]
    # file_name = "{:06d}.png".format(count)
    if n==num_image-1:
        f.write(path_in_split+file_name)
    else:
        f.write(path_in_split+file_name+'\n')


f.close()