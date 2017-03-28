import numpy as np
from operator import and_

def get_mean_values(image, superpixels):
    """ Gets the mean value of each feature dimension of an image 
    for each superpixel.

    Parameters
    ----------
    image: {array-like}, shape: (M, N) or (M, N, n_features)
        An image represented by a ndarray, each of its elements representing 
        a pixel's intensity for each of its feature dimensions.

    superpixels: {array-like}, shape: (M, N)
        An image array where each element has an integer number denoting which 
        superpixel it belongs to.

    Returns
    -------
    X: {array-like}, shape: (n_superpixels, n_features)
        Mean data values describing each superpixel
    """

    if len(image.shape) != 3: # single channeled i.e. 1D feature space
        intensity = np.bincount(superpixels.ravel(), weights=image.ravel())
        mean_values = (intensity 
                       / np.bincount(superpixels.ravel())).reshape(-1,1)
    else: # Multichanneled
        m, n, channels = image.shape
        features = [np.bincount(
            superpixels.ravel(),
            weights=image[:, :, i].ravel()) for i in range(channels)]
        mean_values = (np.vstack(features)
                       / np.bincount(superpixels.ravel())).reshape(-1,channels)
    return mean_values

# http://mail.scipy.org/pipermail/scipy-dev/2009-May/011891.html

def groupmeanbin(factors, values):
    """uses np.bincount, assumes factors/labels are integers

    Code from:
    http://mail.scipy.org/pipermail/scipy-dev/2009-May/011891.html
    """
    ix,rind = np.unique(factors, return_inverse=True)
    gcount = np.bincount(rind)
    gmean = np.bincount(rind, weights=values)/ (1.0*gcount)
    return gmean

def groupstdbin(factors, values):
    """uses np.bincount, assumes factors/labels are integers

    Code from:
    http://mail.scipy.org/pipermail/scipy-dev/2009-May/011891.html
    """
    ix,rind = np.unique(factors, return_inverse=True)
    gcount = np.bincount(rind)
    gmean = np.bincount(rind, weights=values)/ (1.0*gcount)
    meanarr = gmean[rind]
    withinvar = np.bincount(rind, weights=(values-meanarr)**2) / (1.0*gcount)
    return np.sqrt(withinvar)

def get_final_segmentation(image, segmented, y, n):
    """
    Parameters
    ----------
    image: a (M, N) array,
        the original, single-channel image, which was passed into the
        SLIC superpixel algorithm.  For crop images, it is best if this
        image is in NDVI format.

    segmented : a (M, N) array.
        The superpixel segmentation of the image.  Pixel (i,j)'s value is
        the superpixel index it belongs to

    y: a 1-D array,
        where y[i] is the label for superpixel i

    n: number of segments 
        used as a parameter for the clustering algorithm.

    Returns
    -------
    final_segmentation: a list of (M, N) masked arrays,
        where each masked array, final_segmentaiton[i], 
        is the image of the segmented regions which
        belong to cluster/class i
    """

    sp_ids = get_superpixels_belonging_to_labels(y, n)
    p_belongs_to_label = get_pixels_belonging_to_label(segmented, sp_ids, y, n)
    return p_belongs_to_label

def get_superpixels_belonging_to_labels(y, n):
    """
    Parameters
    ----------
    y: a 1-D array,
        where y[i] is the label for superpixel i

    n: number of segments 
        used as a parameter for the clustering algorithm.

    Returns
    -------
    sp_ids: a tuple of 1-D arrays,
        where sp_ids[j] is an array containing the 
        superpixel indexes which belong to label j. 
    """

    sp_ids = ()

    for label in range(n):
        sp_ids += np.where(y==label)
    return sp_ids

def get_pixels_belonging_to_label(segmented, sp_idxs, y, n):
    """
    Parameters
    ----------
    segmented : a (M, N) array.
        The superpixel segmentation of the image.  Pixel (i,j)'s value is
        the superpixel index it belongs to

    sp_idxs: a tuple of 1-D arrays,
        where sp_idxs[j] is an array containing the 
        superpixel indexes which belong to label j. 

    y: a 1-D array,
        where y[i] is the label for superpixel i

    n: number of segments 
        used as a parameter for the clustering algorithm.
    -------
    p_belongs_to_label: a tuple of (M, N) boolean arrays
        where p_belongs_to_label[k] is the boolean array corresponding
        to label k, and the value of pixel (i,j) in one of the arrays
        denotes whether the pixel belongs to the label (True) or not
        (False).
    """
    p_belongs_to_label = [np.in1d(
        segmented.ravel(), sp_idxs[label]).reshape(segmented.shape)
        for label in range(n)] #np.unique(y)]

    return p_belongs_to_label

def and_ma_arrays(*ma_arrays):
    """
    For a masked array, ma1, where unmasked data is superpixel0,
    and another masked array, ma2, where unmasked data is 
    superpixel1, we can logical "and" ma1 and ma2 in order to generate
    a masked array ma1_2, where the unmasked data is now 
    superpixel0 and superpixel1)

    Parameters
    ----------
    ma_arrays: a collection of masked arrays to be "anded" together

    Returns
    -------
    a masked array whose unmasked data is the combined unmasked data
    of the masked arrays in ma_arrays
    """

    data_sum = sum([ma_array.data for ma_array in ma_arrays])
    masks = [ma_array.mask for ma_array in ma_arrays]
    return np.ma.array(data_sum, mask=map(and_, *masks))
