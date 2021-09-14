import numpy as np
from PIL import Image
import cv2
from scipy.ndimage.interpolation import rotate
import scipy.misc
import random
import math

def DepthNorm(x, maxDepth):
    return maxDepth / x

def predict(model, images, minDepth=10, maxDepth=1000, batch_size=2):
    # Support multiple RGBs, one RGB image, even grayscale 
    if len(images.shape) < 3: images = np.stack((images,images,images), axis=2)
    if len(images.shape) < 4: images = images.reshape((1, images.shape[0], images.shape[1], images.shape[2]))
    # Compute predictions
    predictions = model.predict(images, batch_size=batch_size)
    #print(predictions)
    # Put in expected range
    return np.clip(DepthNorm(predictions, maxDepth=maxDepth), minDepth, maxDepth) / maxDepth


def notscale_predict(model, images, batch_size=2):
    # Support multiple RGBs, one RGB image, even grayscale
    if len(images.shape) < 3: images = np.stack((images,images,images), axis=2)
    if len(images.shape) < 4: images = images.reshape((1, images.shape[0], images.shape[1], images.shape[2]))
    # Compute predictions
    predictions = model.predict(images, batch_size=batch_size)
    return predictions


def scale_up(scale, images):
    from skimage.transform import resize
    scaled = []
    
    for i in range(len(images)):
        img = images[i]
        output_shape = (scale * img.shape[0], scale * img.shape[1])
        scaled.append( resize(img, output_shape, order=1, preserve_range=True, mode='reflect', anti_aliasing=True ) )

    return np.stack(scaled)

def load_images(image_files):
    loaded_images = []
    for file in image_files:
        x = np.clip(np.asarray(Image.open( file ), dtype=float) / 255, 0, 1)
        h, w, ch = x.shape
        MINConv = 32
        if h % MINConv != 0 or w % MINConv != 0 and ch == 3:
            resized_h = MINConv - h % MINConv
            resized_w = MINConv - w % MINConv
            x = np.pad(x, [(0, resized_h), (0, resized_w), (0, 0)], 'constant')
        loaded_images.append(x)
    return np.stack(loaded_images, axis=0)

def load_mono_image(file_name):
    x = np.clip(np.asarray(Image.open(file_name), dtype=float) / 255, 0, 1)
    h, w, ch = x.shape
    MINConv = 32
    if h % MINConv != 0 or w % MINConv != 0 and ch == 3:
        resized_h = MINConv - h % MINConv
        resized_w = MINConv - w % MINConv
        x = np.pad(x, [(0, resized_h), (0, resized_w), (0, 0)], 'constant')
    return np.stack(x, axis=0)


def to_multichannel(i):
    if i.shape[2] == 3: return i
    i = i[:,:,0]
    return np.stack((i,i,i), axis=2)
        
def display_images(outputs, inputs=None, gt=None, is_colormap=True, is_rescale=True):
    import matplotlib.pyplot as plt
    import skimage
    from skimage.transform import resize

    plasma = plt.get_cmap('plasma')

    shape = (outputs[0].shape[0], outputs[0].shape[1], 3)
    
    all_images = []

    for i in range(outputs.shape[0]):
        imgs = []
        
        if isinstance(inputs, (list, tuple, np.ndarray)):
            x = to_multichannel(inputs[i])
            x = resize(x, shape, preserve_range=True, mode='reflect', anti_aliasing=True )
            imgs.append(x)

        if isinstance(gt, (list, tuple, np.ndarray)):
            x = to_multichannel(gt[i])
            x = resize(x, shape, preserve_range=True, mode='reflect', anti_aliasing=True )
            imgs.append(x)

        if is_colormap:
            rescaled = outputs[i][:,:,0]
            if is_rescale:
                rescaled = rescaled - np.min(rescaled)
                rescaled = rescaled / np.max(rescaled)
            imgs.append(plasma(rescaled)[:,:,:3])
        else:
            imgs.append(to_multichannel(outputs[i]))

        img_set = np.hstack(imgs)
        all_images.append(img_set)

    all_images = np.stack(all_images)
    
    return skimage.util.montage(all_images, multichannel=True, fill=(0,0,0))

def save_images(filename, outputs, inputs=None, gt=None, is_colormap=True, is_rescale=False):
    montage =  display_images(outputs, inputs, is_colormap, is_rescale)
    im = Image.fromarray(np.uint8(montage*255))
    im.save(filename)

def load_test_data(test_data_zip_file='nyu_test.zip'):
    print('Loading test data...', end='')
    import numpy as np
    from data import extract_zip
    data = extract_zip(test_data_zip_file)
    from io import BytesIO
    rgb = np.load(BytesIO(data['eigen_test_rgb.npy']))
    depth = np.load(BytesIO(data['eigen_test_depth.npy']))
    crop = np.load(BytesIO(data['eigen_test_crop.npy']))
    print('Test data loaded.\n')
    return {'rgb':rgb, 'depth':depth, 'crop':crop}

def compute_errors(gt, pred):
    thresh = np.maximum((gt / pred), (pred / gt))
    a1 = (thresh < 1.25   ).mean()
    a2 = (thresh < 1.25 ** 2).mean()
    a3 = (thresh < 1.25 ** 3).mean()
    abs_rel = np.mean(np.abs(gt - pred) / gt)
    rmse = (gt - pred) ** 2
    rmse = np.sqrt(rmse.mean())
    log_10 = (np.abs(np.log10(gt)-np.log10(pred))).mean()
    return a1, a2, a3, abs_rel, rmse, log_10

def evaluate(model, rgb, depth, crop, batch_size=6, verbose=False):
    N = len(rgb)

    bs = batch_size

    predictions = []
    testSetDepths = []
    for i in range(N//bs):

        x = rgb[(i)*bs:(i+1)*bs,:,:,:]
        """
        print(x.shape)
        flipped = x[:, :, :, :]
        print(flipped.shape)
        flipped = np.rot90(flipped, k=-1, axes=(1, 2))
        print(flipped.shape)
        flipped = np.reshape(flipped, (bs, 640, 480, 3))

        if i == 1:
            from PIL import Image
            aug_img = Image.fromarray(np.clip(np.uint8(flipped * 1), 0, 255))
            aug_img.save('test.jpg', quality=99)
        """

        # Compute results
        true_y = depth[(i)*bs:(i+1)*bs, :, :]

        magnitude = random.randint(-180, 180)
        print('magnitude=', magnitude)
        x = x[0, :, :, :]
        h, w, ch = x.shape
        rotated_x = rotate(x, magnitude, axes=(0, 1), reshape=True)
        h_rot, w_rot, _ = rotated_x.shape
        MINConv = 32
        if h_rot % MINConv != 0 or w_rot % MINConv != 0:
            resized_h = MINConv - h_rot % MINConv
            resized_w = MINConv - w_rot % MINConv
            rotated_x = np.pad(rotated_x, [(0, resized_h), (0, resized_w), (0, 0)], 'constant')

        pred_y = scale_up(2, predict(model, rotated_x / 255, minDepth=10, maxDepth=1000, batch_size=bs)[:, :, :, 0]) * 10.0
        pred_y = pred_y[0, 0:h_rot, 0:w_rot]
        matrix = cv2.getRotationMatrix2D((w_rot / 2, h_rot / 2), -magnitude, 1.0)
        matrix[0, 2] = matrix[0, 2] - w_rot / 2 + w / 2
        matrix[1, 2] = matrix[1, 2] - h_rot / 2 + h / 2
        pred_y = cv2.warpAffine(pred_y, matrix, (w, h))
        """
        gt = 255 * rerolled / np.amax(rerolled)
        gt = np.uint8(gt)
        gt = cv2.applyColorMap(gt, cv2.COLORMAP_JET)
        cv2.imwrite('GT_{}.jpg'.format(i), gt)
        """
        """
        x = np.rot90(x, k=2, axes=(1, 2))
        pred_y_180 = scale_up(2, predict(model, x / 255, minDepth=10, maxDepth=1000, batch_size=bs)[:, :, :, 0]) * 10.0
        pred_y_180 = np.rot90(pred_y_180, k=-2, axes=(1, 2))
        pred_y = pred_y_180[:, crop[0]:crop[1] + 1, crop[2]:crop[3] + 1]
        """

        for j in range(len(true_y)):
            predictions.append(1.0 * pred_y[j])
            testSetDepths.append(true_y[j])


    predictions = np.stack(predictions, axis=0)
    testSetDepths = np.stack(testSetDepths, axis=0)
    print('pred=', predictions.shape)
    print('groundtruth=', testSetDepths.shape)

    e = compute_errors(predictions, testSetDepths)

    if verbose:
        print("{:>10}, {:>10}, {:>10}, {:>10}, {:>10}, {:>10}".format('a1', 'a2', 'a3', 'rel', 'rms', 'log_10'))
        print("{:10.4f}, {:10.4f}, {:10.4f}, {:10.4f}, {:10.4f}, {:10.4f}".format(e[0],e[1],e[2],e[3],e[4],e[5]))

    return e


def evaluate2(model, rgb, depth, crop, batch_size=1, verbose=False):
    N = len(rgb)
    bs = batch_size = 1
    model_name = 'NoDataAugmentation'

    print('Loop num=', range(N // bs))
    resultAbsRel_file = open('./result_AbsRel_' + model_name + '.txt', 'w')
    resultRMSE_file = open('./result_RMSE_' + model_name + '.txt', 'w')
    resultAbsRel_file.write('\n')
    resultRMSE_file.write('\n')

    for i in range(N // bs):

        x = rgb[(i) * bs:(i + 1) * bs, :, :, :]
        # Compute results
        true_y = depth[(i) * bs:(i + 1) * bs, :, :]
        magnitude = 10 * (i % 37) - 180
        if i > 110:
            break;

        #magnitude = random.randint(-180, 180)
        print('magnitude=', magnitude)
        x = x[0, :, :, :]
        h, w, ch = x.shape
        rotated_x = rotate(x, magnitude, axes=(0, 1), reshape=True)
        h_rot, w_rot, _ = rotated_x.shape
        MINConv = 32
        if h_rot % MINConv != 0 or w_rot % MINConv != 0:
            resized_h = MINConv - h_rot % MINConv
            resized_w = MINConv - w_rot % MINConv
            rotated_x = np.pad(rotated_x, [(0, resized_h), (0, resized_w), (0, 0)], 'constant')

        pred_y = scale_up(2, predict(model, rotated_x / 255, minDepth=10, maxDepth=1000, batch_size=bs)[:, :, :, 0]) * 10.0
        pred_y = pred_y[0, 0:h_rot, 0:w_rot]
        matrix = cv2.getRotationMatrix2D((w_rot / 2, h_rot / 2), -magnitude, 1.0)
        matrix[0, 2] = matrix[0, 2] - w_rot / 2 + w / 2
        matrix[1, 2] = matrix[1, 2] - h_rot / 2 + h / 2
        pred_y = cv2.warpAffine(pred_y, matrix, (w, h))

        true_y = true_y[0, :, :]
        true_y = true_y[crop[0]: crop[1] + 1, crop[2]: crop[3] + 1]
        pred_y = pred_y[crop[0]: crop[1] + 1, crop[2]: crop[3] + 1]
        abs_rel = np.mean(np.abs(true_y - pred_y) / true_y)
        rmse = (true_y - pred_y) ** 2
        rmse = np.sqrt(rmse.mean())
        accuracyAbsRel = '< AbsRel >' + ' key = ' + str(i) + ' angle = ' + str(magnitude) + ' NotRollRefined = ' + str(abs_rel)
        accuracyRMSE = '< RMSE >' + ' key = ' + str(i) + ' angle = ' + str(magnitude) + ' NotRollRefined = ' + str(rmse)

        resultAbsRel_file.write(accuracyAbsRel + '\n')
        resultRMSE_file.write(accuracyRMSE + '\n')
    resultAbsRel_file.close()
    resultRMSE_file.close()
