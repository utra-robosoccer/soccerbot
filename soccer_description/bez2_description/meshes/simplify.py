# Requires stl files to be in a folder named 'raw' in the same directory as this script
# Loops through the files and puts a simplified version in a folder named 'simplified'
# The reduction factor is set by the REDUCTION variable (e.g. 0.7 = 30% of the original size)

import os
import glob
import pyvista as pv
from fast_simplification import simplify_mesh

REDUCTION = 0.7

filepaths = glob.glob(os.path.join(os.getcwd(), 'raw', '*'))

if not os.path.exists('simplified'):
    os.makedirs('simplified')

for filepath in filepaths:
    
    print(filepath)
    mesh = pv.read(filepath)

    out = simplify_mesh(mesh, target_reduction=REDUCTION)
    out.save(filepath.replace('raw', 'simplified'))
    print('Simplified', filepath.split('/')[-1])