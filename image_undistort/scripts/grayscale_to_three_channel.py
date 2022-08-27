import cv2
N_IMAGE = 761

for i in range(N_IMAGE):
    input_path = "/data/datasets/rs_vio_rect/cam0_rect/{:06d}.png".format(i)
    img = cv2.imread(input_path, cv2.IMREAD_COLOR)
    output_path = "/home/jy/Desktop/test/{:06d}.png".format(i)
    cv2.imwrite(output_path, img)
    print("finish writing:", output_path)
