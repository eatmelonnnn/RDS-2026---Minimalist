import mujoco

model = mujoco.MjModel.from_xml_path('urdf_exports2.urdf')
mujoco.mj_saveLastXML('finger.xml', model)

print('Conversion successful')