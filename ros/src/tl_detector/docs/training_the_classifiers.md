# Training the Classifier Models
This file is an attempt to document the steps in training the single-shot detection models for traffic light states. To ensure reproducibility, the reader should pay particular attention to their environment setup and adjust all file paths accordingly.

#### 1. Setup
Use TensorFlow-gpu 1.4, CUDA-8.0, CuDNN-6.0 to run training script.
For TensorFlow 1.3 compatibility - branch `1f34fcafc1454e0d31ab4a6cc022102a54ac0f5b`, see:
```
  https://github.com/tensorflow/models/blob/1f34fcafc1454e0d31ab4a6cc022102a54ac0f5b/research/object_detection/train.py

This branch includes TensorFlow pretrained models that were generated with an earlier version of Tensorflow.
```

For TensorFlow's Object Detection API installation instructions, see:
```
  https://github.com/tensorflow/models/blob/1f34fcafc1454e0d31ab4a6cc022102a54ac0f5b/research/object_detection/g3doc/installation.md
```

For instructions to train/fine-tune the pretrained object detection models, see:
```
  https://github.com/smasoudn/traffic_light_detection
```
---

#### 2. Put the data files in the following file structure:
Training datasets are included in the repo referenced [above](https://github.com/smasoudn/traffic_light_detection).

```
.<working_space_folder>
├── data
│   └── sim_data.record
├── pretrained_models
│   └── ssd_inception_v2_coco_2017_11_17
│       ├── checkpoint
│       ├── frozen_inference_graph.pb
│       ├── model.ckpt.data-00000-of-00001
│       ├── model.ckpt.index
│       ├── model.ckpt.meta
│       └── saved_model
│           ├── saved_model.pb
│           └── variables
└── training
   ├── label_map.pbtxt
   └── ssd_inception_v2_coco.config
```

###### NOTE:
I was blocked by a `ValueError: axis = 0 not in [0, 0)` while running train.py. A workaround fix was to add `anchorwise_output: true` to the pipeline config file, <pretrained_model>.config. See stackoverflow question:
```
  https://stackoverflow.com/questions/48847365/tensorflow-object-detection-api-training-error
  or permlink: https://stackoverflow.com/a/49145912
```
---

#### 3. Run training
* From the `tensorflow/models/research/` directory, set the python path
```bash
>  export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
```
* Train the simulator model
```bash
>  python object_detection/train.py --logtostderr --train_dir=object_detection/udacity_capstone_sim/training/ --pipeline_config_path=object_detection/udacity_capstone_sim/training/ssd_inception_v2_coco.config
```
* Train the real-world model
```bash
>  python object_detection/train.py --logtostderr --train_dir=object_detection/udacity_capstone_real/training/ --pipeline_config_path=object_detection/udacity_capstone_real/training/ssd_inception_v2_coco.config
```
---
#### 4. Export the trained model for inference
```bash
>  python object_detection/export_inference_graph.py --input_type image_tensor --pipeline_config_path=object_detection/udacity_capstone_sim/training/ssd_inception_v2_coco.config --trained_checkpoint_prefix=object_detection/udacity_capstone_sim/training/model.ckpt-9931 --output_directory object_detection/udacity_capstone_sim/fine_tuned_models/ssd_inception_v2_coco_ud_capstone_sim
```

```bash
>  python object_detection/export_inference_graph.py --input_type image_tensor --pipeline_config_path=object_detection/udacity_capstone_real/training/ssd_inception_v2_coco.config --trained_checkpoint_prefix=object_detection/udacity_capstone_real/training/model.ckpt-9482 --output_directory object_detection/udacity_capstone_real/fine_tuned_models/ssd_inception_v2_coco_ud_capstone_real
```
###### NOTE:
I was blocked by `ValueError: Protocol message RewriterConfig has no "layout_optimizer" field`.

Here's a hack that got me past this issue ( See: https://github.com/tensorflow/tensorflow/issues/16268 ):

Change models/research/object_detection/exporter.py line 71/72 from:
```python
  rewrite_options = rewriter_config_pb2.RewriterConfig(
          layout_optimizer=rewriter_config_pb2.RewriterConfig.ON)
```
  to
```python
  rewrite_options = rewriter_config_pb2.RewriterConfig()
```
