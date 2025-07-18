# Ball and Robot Detection Package

We are using YOLOv5 model trained with Ultralytics tools.
Checkout their website for steps on training a mode: https://docs.ultralytics.com/tutorials/train-custom-datasets/

Previously we used a custom U-net type model based on Bit Bot's old [paper](https://robocup.informatik.uni-hamburg.de/wp-content/uploads/2018/06/2018_Speck_Ball_Localization.pdf).
We didn't get good detection rate, with false negatives when ball was small and far away. We used below dataset to train the old model. The data set can be reused to train newer models:

### train dataset

1. https://imagetagger.bit-bots.de/images/imageset/13/ #bitbots-set00-05
2. https://imagetagger.bit-bots.de/images/imageset/836/ #sequences-jasper-euro-ball-1
3. https://imagetagger.bit-bots.de/images/imageset/153/ #sequences-euro-ball-robot-1
4. https://imagetagger.bit-bots.de/images/imageset/15/ #bitbots-set00-07
5. https://imagetagger.bit-bots.de/images/imageset/12/ #bitbots-set00-04
6. https://imagetagger.bit-bots.de/images/imageset/18/ #bitbots-set00-10
7. https://imagetagger.bit-bots.de/images/imageset/352/ #imageset_352
8. https://imagetagger.bit-bots.de/images/imageset/168/ #imageset_168
9. https://imagetagger.bit-bots.de/images/imageset/16/ #bitbots-set00-08
10. https://imagetagger.bit-bots.de/images/imageset/61/ #imageset_61
11. https://imagetagger.bit-bots.de/images/imageset/154/ #sequences-misc-ball-1

### test dataset

1. https://imagetagger.bit-bots.de/images/imageset/833/ #test-wolves-01
2. https://imagetagger.bit-bots.de/images/imageset/835/ #test-nagoya-game-02
