from controller import Supervisor

supervisor = Supervisor()
box = supervisor.getFromDef('Box1')  # Use the DEF name of the box
material = box.getField('appearance').getSFNode().getField('material')
material.getField('diffuseColor').setSFVec3f([0.5, 0.0, 0.5])  # Set color to purple
