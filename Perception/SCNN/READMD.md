# SCNN-Based Lane Detection

## 1. Overview

To achieve reliable lane-centered autonomous driving in the QLabs environment, we adopted **Spatial Convolutional Neural Networks (SCNN)** as the core lane detection module.  
SCNN enhances conventional CNNs by propagating spatial information along rows and columns, enabling robust detection of continuous lane structures under curvature, discontinuities, and partial occlusions.

The network outputs a **binary lane segmentation mask**, which is post-processed to extract left/right lane boundaries and a **drivable centerline**.  
This centerline represents a high-level interpretation of road geometry and is shared with downstream modules for planning and control.

<img src="/images/SCNN1.gif" width="800"/>

---

## 2. Why SCNN?

Classical vision-based lane detection methods (thresholding, edge detection, Hough transform) are highly sensitive to illumination changes and texture noise.

SCNN was selected because it provides:

- Strong spatial continuity on curved and dashed lanes  
- Robust performance under partial occlusion  
- Segmentation-based output suitable for geometric post-processing  
- Real-time feasibility for ROS2-based autonomous pipelines  

<img src="/images/SCNN2.png" width="800"/>

---

## 3. Domain Adaptation via Fine-Tuning

Applying SCNN models pretrained on large-scale real-world datasets directly to the QLabs environment resulted in degraded performance due to **domain mismatch** (scale, texture, lighting, and camera perspective).

To address this, we performed **domain adaptation through fine-tuning**:

- Low-level layers preserve generic edge and texture features  
- Higher-level layers are fine-tuned using **QLabs-specific lane data**  
- This improves lane continuity and stability in the target domain

<p align="center">
  <img src="/images/SCNN3.png" width="500"/>
</p>

Feature map inspection during training showed that:
- Early layers remained largely unchanged  
- Later layers increasingly specialized to lane boundaries and curvature


<p align="center">
  <img src="/images/SCNN4.png" width="600"/>
</p>
---

## 4. Dataset Construction

The dataset was constructed directly from QLabs driving scenarios.

- Input: RGB images from a forward-facing camera  
- Labels: Binary segmentation masks  
  - Lane pixels = 1  
  - Background = 0  
- Only lane markings are annotated to focus perception on road geometry  
- Split: 80% training, 20% validation  


<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="images/4SCNN.png" width="300">
    </td>
    <td align="center">
      <img src="images/1SCNN.png" width="300">
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="images/2SCNN.png" width="300">
    </td>
    <td align="center">
      <img src="images/3SCNN.png" width="300">
    </td>
  </tr>
</table>

---

## 5. Training Pipeline

**Network**
- Input resolution: 480×640 RGB  
- Backbone: VGG16 with batch normalization  
- SCNN spatial propagation layers  
- 1-channel output logit map

**Training setup**
- Loss: `BCEWithLogitsLoss`  
- Optimizer: AdamW with weight decay  
- Metric: Mean Intersection over Union (mIoU)  

Training curves demonstrated stable convergence after fine-tuning.


<p align="center">
  <img src="/images/SCNN5.png" width="400"/>
</p>


---

## 6. Post-processing and Lane Interpretation

The raw SCNN output mask is transformed into a structured lane representation:

1. Morphological filtering  
2. Row-wise lane point extraction  
3. Clustering to separate left and right lanes  
4. Polynomial or spline fitting  
5. Centerline computation  
6. Temporal smoothing  

| Before | After |
|---|---|
| ![](/images/SCNN7.png) | ![](/images/SCNN8.png) |

This process converts pixel-level perception into system-level lane geometry suitable for autonomous driving.

---

## 7. Integration with the Autonomous Stack

The extracted centerline is interpreted as a sequence of waypoints:

- Combined with the vehicle pose estimate  
- Transformed into a consistent coordinate frame  
- Published via ROS2 topics for downstream planning and control  

<p align="center">
  <img src="/images/SCNN9.png" width="800"/>
</p>
---

## 8. Evaluation Results

**Qualitative**
- Stable detection on straight and curved roads  
- Continuous centerline generation on dashed lanes  

**Quantitative**
- Validation mIoU consistently in the mid-0.8 range  


<img src="/images/SCNN1.gif" width="800"/>

---

## 9. Summary

By fine-tuning SCNN on QLabs-specific data, we adapted a deep lane detection model to the competition environment.  
The system converts raw camera data into a structured centerline representation, demonstrating readiness in terms of perception, interpretation, and system integration.

