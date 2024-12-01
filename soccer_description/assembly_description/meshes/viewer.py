# Views all stl files in a folder

import os
import glob
import pyvista as pv

FOLDER_NAME = 'simplified'
filepaths = glob.glob(os.path.join(os.getcwd(), FOLDER_NAME, '*'))

for filepath in filepaths:
    
    print(filepath)
    mesh = pv.read(filepath)
    mesh.plot()