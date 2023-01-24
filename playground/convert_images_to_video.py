# put to the folder with images to convert
import cv2
import glob
import os


def main():

    print('Conversion to video...')

    # collecting every frame in an array
    img_array = []
    for filename in sorted(glob.glob('*.png')):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)
        #os.remove(filename) #uncommenting will remove frames after conversion

    # re-creating frames as a video in '.mp4' format
    out = cv2.VideoWriter('converted_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 2, size)
    for image in img_array:
        out.write(image)
    out.release()
    
    print('Conversion completed.')
    

if __name__ == '__main__':
    main()
