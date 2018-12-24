# TrackingMovingObjects_PIV
Work developed for the Image Processing and Vision course from Instituto Superior Técnico.

# Description
  The goal of the project is the detection, localization and tracking of moving objects using information from depth cameras.
  The cameras are in a fixed position and the scene is composed of a static set of objects (background) and a variable number of moving objects. 

## Part 1
  - Considering just one camera for each detected object in each frame a box must be draw around it.
  - **Output:** for every tracked object a struct called `objects` is returned, with 4 fields the first one being the `X` coordinates of the box, the second the `Y` coordinates of the box, the third the `Z` coordinates of the box and a fourth one called `frames_tracked` containg the frames where the objects were detected.

## Part 2
  - Considering two cameras, for each detected object (no matter the camera) in each frame a box containing the identified object must be draw.
  - **Output:** same as the part 1 plus another struct called `cam2toW` containing the rotacional and translation matrices of the camera 2 in relation to the world coordinate system(camera 1).
  
# Notes 
 For part 1 use the files in the `images1` or `images4` folders. Only change the name of the folder on the top of the file `part1_test`.
 For part 2 use the files in the `images2` or `images3`**(never tested)** folders. Pay attencion to change the name of the folder in the top of the file `part2_test`.
  
# Done by:
  - Luís Lopes
  - Diogo Morgado
  - Valter Piedade
