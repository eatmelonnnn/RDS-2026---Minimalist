import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import time
from pydrake.all import *

# Initialization
meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step = 0.0)
parser = Parser(plant=plant, scene_graph=scene_graph)
parser.SetAutoRenaming(True)

# --------------------------------------- ASSEMBLY ----------------------------------------------------
# File paths
base_path = r"minimalist finger sdfs//FingerBase.sdf"
mcp_path = r"minimalist finger sdfs//FingerMCP.sdf"
dowel_pin465_path = r"minimalist finger sdfs//StainlessSteelDowelPin465.sdf"
tendon_pulley_path = r"minimalist finger sdfs//FlexionTendonPulley.sdf"
fingerlink_path = r"minimalist finger sdfs//FingerLink.sdf"
fingertip_phalange_path = r"minimalist finger sdfs//FingertipPhalange.sdf"
# ----------------------------------------- BASE -----------------------------------------------------

# Add base model to plant
base_model = parser.AddModels(base_path)[0]

# Get Base Body
base_body = plant.GetBodyByName("FingerBase", base_model)

# Get Frames
world = plant.world_frame()
base_frame = base_body.body_frame()

# Fix base to world frame
R1 = RotationMatrix([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]) # rotation about y by 180
R2 = RotationMatrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) # rotation about x by 90

base_to_world_T = RigidTransform(R=R1@R2, 
                                p=[0, 0, 0.016]) # note: rotation about z by 180   0.018, 0.015, 0
weldjoint_world = plant.WeldFrames(world, base_frame, base_to_world_T)

R3 = RotationMatrix.MakeYRotation(np.pi)
R4 = RotationMatrix.MakeXRotation(np.pi/2)

# This is a base frame that is aligned with the world frame
base_frame_offset = plant.AddFrame(FixedOffsetFrame("base_frame_offset",
                                                   base_frame,
                                                   RigidTransform(R=R3@R4, p=[0, 0.016, 0])))

# ----------------------------------------- MCP ---------------------------------------------------------
# Get MCP model
mcp_model = parser.AddModels(mcp_path)[0]

# Get MCP body
mcp_body = plant.GetBodyByName("FingerMCP", mcp_model)

# Get MCP Frame
mcp_frame = mcp_body.body_frame()

ir = RotationMatrix.Identity()
Rx_neg_90 = RotationMatrix.MakeXRotation(-np.pi/2)
Rz_neg_90 = RotationMatrix.MakeZRotation(-np.pi/2)
offset_frame_mcp = plant.AddFrame(FixedOffsetFrame("offset_frame_mcp",
                                                   mcp_frame,
                                                   RigidTransform(R=Rx_neg_90@Rz_neg_90, p=[0, 0, -0.02986])))

# Add revolute joint base to mcp
X_PF = base_frame_offset
X_CF = offset_frame_mcp

revolute1 = RevoluteJoint("splay_mcp", X_PF, X_CF, [0, 0, 1], -0.174533, 0.174533, 0)
mcp_splay = plant.AddJoint(revolute1)

# Add Splay Actuator
splay_mcp_joint = plant.GetJointByName("splay_mcp")
splay_actuator = plant.AddJointActuator("splay_actuator", splay_mcp_joint, 1)

# ----------------------------------------- DOWEL PINS 465 ------------------------------------------------
dowel_pin465_model = parser.AddModels(dowel_pin465_path)[0]
dowel_pin465_body = plant.GetBodyByName("StainlessSteelDowelPin465", dowel_pin465_model)
dowel_pin465_frame = dowel_pin465_body.body_frame()

offset_frame_dowelpin465 = plant.AddFrame(FixedOffsetFrame("offset_frame_dowelpin465",
                                                           dowel_pin465_frame,
                                                           RigidTransform(R=Rz_neg_90, p=[0, 0.02986, 0])))

X_PF = offset_frame_mcp
X_CF = offset_frame_dowelpin465

dowelpin465_weld = plant.WeldFrames(X_PF, X_CF, RigidTransform())

# ----------------------------------------- DOWEL PIN 465 MCP --------------------------------------------
dowel_pin465_model1 = parser.AddModels(dowel_pin465_path)[0]
dowel_pin465_body1 = plant.GetBodyByName("StainlessSteelDowelPin465", dowel_pin465_model1)
dowel_pin465_frame1 = dowel_pin465_body1.body_frame()

offset_frame_dowelpin465_1 = plant.AddFrame(FixedOffsetFrame("offset_frame_dowelpin465_1",
                                                           dowel_pin465_frame1,
                                                           RigidTransform(R=Rz_neg_90, p=[0, 0.01486, -0.013])))

X_CF = offset_frame_dowelpin465_1

dowelpin465_weld1 = plant.WeldFrames(X_PF, X_CF, RigidTransform())

# ----------------------------------------- Pulleys mcp side ---------------------------------------------
tendon_pulley1 = parser.AddModels(tendon_pulley_path)[0]
tendon_pulley_body1 = plant.GetBodyByName("FlexionTendonPulley", tendon_pulley1)
tendon_pulley_frame1 = tendon_pulley_body1.body_frame()

offset_tendon_pulley1 = plant.AddFrame(FixedOffsetFrame("offset_tendon_pulley1",
                                                           tendon_pulley_frame1,
                                                           RigidTransform(R=Rz_neg_90, p=[0, -0.0085, 0])))
X_CF = offset_tendon_pulley1
X_PF = dowel_pin465_frame

pulley1_weld = plant.WeldFrames(X_PF, X_CF, RigidTransform())

tendon_pulley2 = parser.AddModels(tendon_pulley_path)[0]
tendon_pulley_body2 = plant.GetBodyByName("FlexionTendonPulley", tendon_pulley2)
tendon_pulley_frame2 = tendon_pulley_body2.body_frame()

offset_tendon_pulley2 = plant.AddFrame(FixedOffsetFrame("offset_tendon_pulley2",
                                                           tendon_pulley_frame2,
                                                           RigidTransform(R=Rz_neg_90, p=[0, 0.0085, 0])))
X_CF = offset_tendon_pulley2
X_PF = dowel_pin465_frame

pulley1_weld = plant.WeldFrames(X_PF, X_CF, RigidTransform())

# ----------------------------------------- FINGER LINK ---------------------------------------------
fingerlink = parser.AddModels(fingerlink_path)[0]
fingerlink_body = plant.GetBodyByName("FingerLink", fingerlink)
fingerlink_frame = fingerlink_body.body_frame()

Rz_pos_180 = RotationMatrix.MakeZRotation(np.pi)
Rx_pos_90 = RotationMatrix.MakeXRotation(np.pi/2)
Ry_pos_90 = RotationMatrix.MakeYRotation(np.pi/2)
offset_fingerlink = plant.AddFrame(FixedOffsetFrame("offset_fingerlink",
                                                    fingerlink_frame, 
                                                    RigidTransform(R=Rz_neg_90@Ry_pos_90, p=[0,0,0])))
X_CF = offset_fingerlink # 0.02985
X_PF = dowel_pin465_frame

revolute2 = RevoluteJoint("flexion_mcp", X_PF, X_CF, np.array([1, 0, 0]), 1.5708, 3.14159, 0)
mcp_flexion = plant.AddJoint(revolute2)

# Add MCP Flexion Actuator
flexion_mcp_joint = plant.GetJointByName("flexion_mcp")
flexion_mcp_actuator = plant.AddJointActuator("flexion_mcp_actuator", flexion_mcp_joint, 1.2)

# ----------------------------------------- DOWELPIN 465 PIP ---------------------------------------------
dowel_pin_pip = parser.AddModels(dowel_pin465_path)[0]
dowel_pin_pip_body = plant.GetBodyByName("StainlessSteelDowelPin465", dowel_pin_pip)
dowel_pin_pip_frame = dowel_pin_pip_body.body_frame()

offset_dowel_pin_pip  = plant.AddFrame(FixedOffsetFrame("offset_dowel_pin_pip",
                                                    dowel_pin_pip_frame, 
                                                    RigidTransform(R=ir, p=[0,0.043,0])))

X_CF = offset_dowel_pin_pip
X_PF = offset_fingerlink

dowel_pin_pip_weld = plant.WeldFrames(X_PF, X_CF, RigidTransform())

# ----------------------------------------- DOWELPIN 465 PIP ---------------------------------------------
tendon_pulley_pip = parser.AddModels(tendon_pulley_path)[0]
tendon_pulley_pip_body = plant.GetBodyByName("FlexionTendonPulley", tendon_pulley_pip)
tendon_pulley_pip_frame = tendon_pulley_pip_body.body_frame()

offset_tendon_pulley_pip = plant.AddFrame(FixedOffsetFrame("offset_dowel_pin_pip",
                                                    tendon_pulley_pip_frame, 
                                                    RigidTransform(R=Rz_neg_90, p=[0.043, -0.0085, 0])))

X_CF = offset_tendon_pulley_pip
X_PF = offset_dowel_pin_pip

pulley_pip_weld = plant.WeldFrames(X_PF, X_CF, RigidTransform())

# ----------------------------------------- FINGERTIP PHALANGE ---------------------------------------------
fingertip_phalange = parser.AddModels(fingertip_phalange_path)[0]
fingertip_phalange_body = plant.GetBodyByName("FingertipPhalange", fingertip_phalange)
fingertip_phalange_frame = fingertip_phalange_body.body_frame()

offset_fingertip_phalange = plant.AddFrame(FixedOffsetFrame("offset_fingertip_phalange",
                                                    fingertip_phalange_frame, 
                                                    RigidTransform(R=Rz_neg_90@Ry_pos_90, p=[0, 0, 0])))

X_CF = offset_fingertip_phalange
X_PF = dowel_pin_pip_frame

revolute3 = RevoluteJoint("flexion_pip", X_PF, X_CF, np.array([1, 0, 0]), 1.5708, 3.14159, 0)
flexion_pip = plant.AddJoint(revolute3)

# Add PIP Flexion Actuator
flexion_pip_joint = plant.GetJointByName("flexion_pip")
flexion_pip_actuator = plant.AddJointActuator("flexion_pip_actuator", flexion_pip_joint, 1.0)

# ---------------------------------------VISUALIZATION AND SIMULATION --------------------------------------

# Visualizing plant
plant.Finalize()
visualizer1 = VisualizationConfig()
visualizer1.publish_contacts = False
visualizer1.publish_proximity = False
visualizer1.publish_inertia = False

#   AddDefaultVisualization(builder, meshcat)
ApplyVisualizationConfig(visualizer1, builder, meshcat=meshcat)


diagram = builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyMutableContextFromRoot(context)

diagram.ForcedPublish(context)

def direct_drive_actuation(simulator, plant_context, dip_actuator, pip_actuator, splay_actuator, dip_torque=1, pip_torque=1.2, splay_torque=0.8):
    '''
    Available_integration_schemes: 
    ['bogacki_shampine3', 'cenic', 'explicit_euler', 
    'implicit_euler', 'radau1', 'radau3', 'runge_kutta2', 'runge_kutta3', 'runge_kutta5', 
    'semi_explicit_euler', 'velocity_implicit_euler']
    '''
    simulator_config = SimulatorConfig(accuracy=1.0e-4,
                                       integration_scheme="runge_kutta3",
                                       use_error_control=True,
                                       max_step_size=0.1,
                                       publish_every_time_step=True,
                                       start_time=0.0,
                                       target_realtime_rate = 0.0)
    simulator.Initialize()
    ApplySimulatorConfig(simulator_config)
    simulator.set_monitor(plant_context)



quit = input('Press Enter to quit')

