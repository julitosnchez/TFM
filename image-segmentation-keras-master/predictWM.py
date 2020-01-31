import keras_segmentation

model = keras_segmentation.pretrained.pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset

#model = keras_segmentation.pretrained.pspnet_101_cityscapes() # load the pretrained model trained on Cityscapes dataset

#model = keras_segmentation.pretrained.pspnet_101_voc12() # load the pretrained model trained on Pascal VOC 2012 dataset

# load any of the 3 pretrained models

out = model.predict_segmentation(
    inp="3.jpg",
    out_fname="camera.png"
)
