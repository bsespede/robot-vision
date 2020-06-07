import argparse
import PIL.Image as pil
import cv2
import os
import numpy as np

import src.monodepth2.test_simple as compute
import src.monodepth2.evaluate_depth as evaluate


def compute_depth_maps(input_path, model_name, extension):
    # run the CNN
    args = argparse.Namespace(image_path=input_path, model_name=model_name, ext=extension)
    compute.test_simple(args)


def compare_depth_maps(input_path, groundtruth_path):
    # values taken from 'evaluate_depth' script
    MIN_DEPTH = 1e-3
    MAX_DEPTH = 80
    STEREO_SCALE_FACTOR = 5.4

    for filename in os.listdir(groundtruth_path):
        # load groundtruth depth data
        gt_depth_path = os.path.join(groundtruth_path, filename)
        gt_depth = (np.array(pil.open(gt_depth_path)).astype(np.float32) / 256).astype(np.float32)
        gt_depth = np.clip(gt_depth, 0, MAX_DEPTH)
        gt_height, gt_width = gt_depth.shape[:2]

        # load disparity and convert to depth
        pred_filename = filename.replace("groundtruth_depth", "image").replace(".png", "_disp.npy")
        pred_path = os.path.join(input_path, pred_filename)
        pred_disp = np.load(pred_path)[0][0]
        pred_disp = cv2.resize(pred_disp, (gt_width, gt_height))
        pred_depth = 1 / pred_disp

        # compute mask
        mask = gt_depth > 0
        pred_depth = pred_depth[mask]
        gt_depth = gt_depth[mask]

        # apply median ratio scaling
        pred_depth *= STEREO_SCALE_FACTOR
        ratio = np.median(gt_depth) / np.median(pred_depth)
        pred_depth *= ratio

        # clip to depth range
        pred_depth[pred_depth < MIN_DEPTH] = MIN_DEPTH
        pred_depth[pred_depth > MAX_DEPTH] = MAX_DEPTH

        abs_rel, sq_rel, rmse, rmse_log, a1, a2, a3 = evaluate.compute_errors(gt_depth, pred_depth)
        print(filename, ':', rmse)


if __name__ == "__main__":
    input_path = "../data/images"
    groundtruth_path = "../data/groundtruth"
    model_name = "mono+stereo_1024x320"
    extension = "png"

    #compute_depth_maps(input_path, model_name, extension)
    compare_depth_maps(input_path, groundtruth_path)
