# Welcome to my triangle raytracing renderer

## Requirements:
```cv2``` library from OpenCV (used for saving and displaying images)

```numpy``` library from NumPy (used for creating the image buffer)

### Installing required packages:
cv2: ```pip3 install opencv-python```

numpy: ```pip3 install numpy```

## Renderer v0.1.0 features:
* Triangle-ray intersection calculation
* Baseline

## Renderer v1.2.0:
* Supports STL file input
* Has a MUCH faster rendering engine
* Does a 2D projection on the screen and then ray traces from the triangle to the ray origin
* Has some artefacts like little holes in triangles (probably due to the delta U and delta V)
* Has STL support
* Has no face occlusion

## Renderer v3.0.0:
### ```basic.py```:
* Re-write of the whole program, making more extensive use of functions to improve readability and scalability
* Very slow
* Implements basic optimizations with the determinants (page 10 of the TM)

### ```HDRI.py```:
* Same as previous but adds the support for 360° images for envorinnement texture

## Renderer v4.0.0:
### ```bubble_opt.py```:
* Big stuff, added big optimizations as described in page 13 of TM
* The program responsible for building the optimization tree is ```bubble_search_v2.py```
