#!/usr/bin/env python
"""
This is a library of functions for performing
color-based image segmentation of an image.
"""

import cv2
import numpy as np


def image_one_to_three_channels(img):
    """ Transforms an image from two channels to three channels """
    # First promote array to three dimensions,
    # then repeat along third dimension
    img_three = np.tile(img.reshape(img.shape[0], img.shape[1], 1), (1, 1, 3))
    return img_three


def classifier_parameters():
    '''
    Parameters for the classifier
    '''
    bound_low = (125, 125, 125)
    bound_high = (255, 255, 255)
    return bound_low, bound_high


def pixel_count(img):
    """
    Count how many pixels in the image are non-zero (positive label)
    and zero (negative label)
    """
    nb_positive = np.count_nonzero(img)
    nb_negative = img.size - nb_positive
    return nb_positive, nb_negative


def pixel_count_segmentation(filename):
    """
    Total/positive/negative counts for given image
    """
    bound_low, bound_high = classifier_parameters()

    img = cv2.imread(filename)
    img_seg = cv2.inRange(img, bound_low, bound_high)
    nb_positive, nb_negative = pixel_count(img_seg)
    return img_seg.size, nb_positive, nb_negative


def precision_recall(true_positive, false_positive, false_negative):
    """
    Return precision and recall given counts
    """
    recall = float(true_positive) / (true_positive + false_negative)
    total_positive = true_positive + false_positive
    if total_positive == 0:
        precision = 1
    else:
        precision = float(true_positive) / total_positive
    return precision, recall


def segmentation_statistics(filename_positive, filename_negative):
    """
    Print segmentation statistics for two files with known segmentation
    """
    total_positive, true_positive, false_negative = pixel_count_segmentation(
        filename_positive)
    total_negative, false_positive, true_negative = pixel_count_segmentation(
        filename_negative)

    precision, recall = precision_recall(true_positive, false_positive,
                                         false_negative)

    print("Nb. of positive examples:", total_positive)
    print("Nb. of negative examples:", total_negative)
    print("True positives:", true_positive)
    print("False positives:", false_positive)
    print("False negatives:", false_negative)
    print("True negatives:", true_negative)
    print("Precision:", precision)
    print("Recall:", recall)

    return precision, recall


def image_centroid_horizontal(img):
    """
    Compute the horizontal centroid of white pixels in
    a grascale image
    """
    # img contains only black and white pixels
    total_x = np.array(np.where(img > 0))
    total_x = total_x[1, :]
    if len(total_x) == 0:
        x_centroid = 0
    else:
        x_centroid = int(np.median(total_x))
    return x_centroid


def image_line_vertical(img, x):
    """ Adds a green 3px vertical line to the image """
    cv2.line(img, (x, 0), (x, img.shape[1]), (0, 255, 0), 3)
    return img


def image_segment(my_image, lower_bound, upper_bound):
    location_image = 255 - cv2.inRange(my_image, lower_bound, upper_bound)
    return location_image


def image_mix(robot_image, background_image, lower_bound, upper_bound):
    location_image = image_segment(robot_image, lower_bound, upper_bound)

    mask = np.where(location_image)
    mixed_image = robot_image.copy()
    mixed_image[mask] = background_image[mask]
    return mixed_image
