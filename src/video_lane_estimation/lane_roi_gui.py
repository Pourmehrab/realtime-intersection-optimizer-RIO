import pylab as pl
from roipoly import roipoly 
from skimage import io
import argparse
import csv

def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=str, default="")
    parser.add_argument('--n-lanes', type=int, default=1)
    parser.add_argument('--intersection', type=str, default="")

    args = vars(parser.parse_args())

    if args['image'] == "":
        print(" [!] Provide an image of the intersection with lanes clearly visible")
        exit(1)

    img = pl.imread(args['image'])
    
    # show the image
    pl.imshow(img, interpolation='nearest', cmap="Greys")
    #pl.colorbar()
    pl.title("left click: line segment         right click: close region")
    
    ROIs = []
    roicolors = ['r', 'b', 'g']
    if args['intersection'] != "":
        outfilename = "{}_lane_rois.csv".format(args['intersection'])
    else:
        outfilename = "lane_rois.csv"
    with open("/tmp/sensible/{}".format(outfilename), 'w') as f:
        fieldnames = ['lane','x','y']
        csvfile = csv.DictWriter(f, fieldnames)
        csvfile.writeheader()
        for i in range(args['n_lanes']):
            # User draws the ROI
            ROIs.append(roipoly(roicolor=roicolors[i % 3]))

            for x,y in zip(ROIs[-1].allxpoints, ROIs[-1].allypoints):
                print("lane: {}, ({},{})".format(i+1,x,y))
                csvfile.writerow({'lane': i+1, 'x': x, 'y': y})

            # show the image with the ROI
            pl.imshow(img, interpolation='nearest', cmap="Greys")
            [x.displayROI() for x in ROIs]
            pl.title('lane ROIs')

