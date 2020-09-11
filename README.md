# Welcome to my triangle raytracing renderer

## Renderer v0.1.0 features:
* Triangle-ray intersection calculation
* Baseline

### Renderer v0.1.1:
* Got fully optimised

### Renderer v0.2.0 features:
* Has support for STL file as input
* Shows all faces (no occlusion)

### Renderer v0.2.1:
* Has face occlusion

### Renderer v0.2.2:
* Some optimisations

## Renderer v1.0.0:
* TODO: add T > 0 checking to avoid drawing triangles that are behind
* Has a MUCH faster rendering engine
- does a 2D projection on the screen and then ray traces from the triangle to the ray origin
* Has some artefacts like little holes in triangles (probably due to the delta U and delta V)
* Has STL support
* Has no face occlusion
