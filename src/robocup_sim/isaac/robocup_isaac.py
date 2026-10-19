# Isaac Sim strana simulacie: nacita arenu z USD, importuje robota shoddy
# z xacro v robocup_description a napoji ho na ROS 2:
#   von: /clock, /odom + TF odom->base_link, /scan_raw (LaserScan, frame
#        lidar_frame; robocup.launch.py ho preposiela na /scan)
#   dnu: /cmd_vel
#
# Podvozok je kinematicky: cmd_vel sa nastavi priamo ako rychlost tela a kolesa
# su bez trenia. Fyzikalny 4-kolesovy skid steer v PhysX s tymto URDF
# vibroval, nevedel sa otocit na mieste a prevracal sa.
#
# Spusta ho ../../../run.sh. Rucne cez Python Isaacu zo shellu BEZ
# /opt/ros/humble (bridge ma vlastne Humble kniznice pre Python 3.12):
#   env -i HOME=$HOME DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY PATH=/usr/bin:/bin \
#     ~/isaacsim/python.sh src/robocup_sim/isaac/robocup_isaac.py
import argparse
import math
import os
import shutil
import subprocess
import traceback
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DEFAULT_WORLD = os.path.join(
    WS, 'src/robocup_sim/worlds/robocup_arena/2026_09_01_RoboCup_HCC_Arena_V1.usd')

parser = argparse.ArgumentParser()
parser.add_argument('--world', default=DEFAULT_WORLD, help='USD prostredie')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--x', type=float, default=0.0)
parser.add_argument('--y', type=float, default=0.0)
parser.add_argument('--yaw', type=float, default=0.0)
parser.add_argument('--frames', type=int, default=0,
                    help='Skonci po tolkych snimkach (0 = bez konca)')
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({'headless': args.headless})

import isaacsim.core.experimental.utils.app as app_utils

for ext in ('omni.scene.optimizer.core', 'isaacsim.robot.schema', 'isaacsim.ros2.bridge'):
    app_utils.enable_extension(ext)
simulation_app.update()

import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.ros2.nodes import (Ros2ClockGraphConfig, Ros2RtxLidarGraphConfig,
                                 create_ros2_clock_graph, create_ros2_rtx_lidar_graph)
from isaacsim.sensors.experimental.rtx import Lidar
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

PHYSICS_DT = 1.0 / 60.0
MAX_ACCELERATION = 2.0
MAX_ANGULAR_ACCELERATION = 2 * MAX_ACCELERATION / 0.48
CMD_VEL_NODE = '/Graph/ROS_Base/CmdVel'
BASE_HEIGHT = 0.115 + 0.05 + 0.005


def build_world(stage, usd_file):
    UsdLux.DomeLight.Define(stage, '/World/DomeLight').CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(2000.0)
    UsdGeom.XformCommonAPI(sun).SetRotate((30.0, 20.0, 0.0))

    ground = UsdGeom.Cube.Define(stage, '/World/ground')
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.05))
    UsdGeom.XformCommonAPI(ground).SetScale((25.0, 25.0, 0.05))
    ground.CreateDisplayColorAttr([(0.6, 0.6, 0.6)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    load_usd_world(stage, usd_file)


def load_usd_world(stage, usd_file):
    src = Usd.Stage.Open(usd_file)
    scale = UsdGeom.GetStageMetersPerUnit(src)
    y_up = UsdGeom.GetStageUpAxis(src) == UsdGeom.Tokens.y
    env = UsdGeom.Xform.Define(stage, '/World/environment')
    stage.DefinePrim('/World/environment/asset').GetReferences().AddReference(usd_file)
    xf = UsdGeom.XformCommonAPI(env)
    xf.SetScale((scale, scale, scale))
    if y_up:
        xf.SetRotate((90.0, 0.0, 0.0))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']) \
        .ComputeWorldBound(env.GetPrim()).ComputeAlignedRange()
    center = (box.GetMin() + box.GetMax()) / 2
    xf.SetTranslate((-center[0], -center[1], -box.GetMin()[2]))
    meshes, boxed = 0, []
    xform_cache = UsdGeom.XformCache()
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
    for prim in Usd.PrimRange(env.GetPrim()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        world = xform_cache.GetLocalToWorldTransform(prim)
        axes = [world.GetRow3(i).GetLength() for i in range(3)]
        if max(axes) > 10 * min(axes):
            r = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            cube = UsdGeom.Cube.Define(stage, f'/World/environment_colliders/box_{len(boxed)}')
            UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d((r.GetMin() + r.GetMax()) / 2))
            UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(*(r.GetSize() / 2)))
            cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            boxed.append(f'{prim.GetName()} {r.GetSize()[0]:.2f}x{r.GetSize()[1]:.2f}x{r.GetSize()[2]:.2f}m')
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        meshes += 1
    for prim in Usd.PrimRange(env.GetPrim()):
        if prim.GetName().lower() == 'ceiling':
            UsdGeom.Imageable(prim).MakeInvisible()
    size = box.GetMax() - box.GetMin()
    ViewportManager.set_camera_view('/OmniverseKit_Persp', eye=[0.0, -0.9 * size[1], 0.9 * size[1]],
                                    target=[0.0, 0.0, 0.0])
    print(f'[robocup_isaac] {usd_file}: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} m, '
          f'{meshes} mesh colliders, box colliders for {boxed}')


def import_robot():
    out_dir = os.path.expanduser('~/.cache/robocup_isaac')
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)
    urdf = os.path.join(out_dir, 'shoddy.urdf')
    xacro_file = os.path.join(WS, 'src/robocup_description/urdf/robot.urdf.xacro')
    subprocess.run(
        ['bash', '-c', f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash '
                       f'&& xacro {xacro_file} > {urdf}'],
        env={'HOME': os.path.expanduser('~'), 'PATH': '/usr/bin:/bin'},
        check=True)

    cfg = URDFImporterConfig()
    cfg.urdf_path = urdf
    cfg.usd_path = out_dir
    cfg.fix_base = False
    cfg.merge_fixed_joints = True
    cfg.ros_package_paths = [{'name': 'robocup_description',
                              'path': os.path.join(WS, 'src/robocup_description')}]
    cfg.joint_target_type = 'none'
    usd_file = URDFImporter(cfg).import_urdf()
    if not usd_file:
        raise RuntimeError(f'URDF import failed for {urdf}')
    return usd_file, lidar_offset(urdf)


def lidar_offset(urdf):
    for joint in ET.parse(urdf).getroot().iter('joint'):
        if joint.find('child').get('link') == 'lidar_frame':
            assert joint.find('parent').get('link') == 'base_link'
            return [float(v) for v in joint.find('origin').get('xyz').split()]
    raise RuntimeError(f'no joint to lidar_frame in {urdf}')


def find_prims(root_prim):
    articulation = base_link = None
    for prim in Usd.PrimRange(root_prim):
        if articulation is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation = prim.GetPath().pathString
        if prim.GetName() == 'base_link' and base_link is None:
            base_link = prim.GetPath().pathString
    if not (articulation and base_link):
        raise RuntimeError(f'robot prims not found: articulation={articulation} '
                           f'base_link={base_link}')
    return articulation, base_link


def make_frictionless(stage, robot):
    material = UsdShade.Material.Define(stage, '/World/PhysicsMaterials/frictionless')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.0)
    physics_material.CreateDynamicFrictionAttr(0.0)
    physics_material.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(robot).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics')


def create_base_graph(articulation):
    keys = og.Controller.Keys
    target = [usdrt.Sdf.Path(articulation)]
    og.Controller.edit(
        {'graph_path': '/Graph/ROS_Base', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('Tick', 'omni.graph.action.OnPlaybackTick'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('CmdVel', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                ('Odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
            ],
            keys.SET_VALUES: [
                ('SimTime.inputs:resetOnStop', True),
                ('CmdVel.inputs:topicName', '/cmd_vel'),
                ('Odom.inputs:chassisPrim', target),
                ('PubOdom.inputs:topicName', '/odom'),
                ('PubOdom.inputs:odomFrameId', 'odom'),
                ('PubOdom.inputs:chassisFrameId', 'base_link'),
                ('PubTF.inputs:parentFrameId', 'odom'),
                ('PubTF.inputs:childFrameId', 'base_link'),
            ],
            keys.CONNECT: [
                ('Tick.outputs:tick', 'CmdVel.inputs:execIn'),
                ('Tick.outputs:tick', 'Odom.inputs:execIn'),
                ('Context.outputs:context', 'CmdVel.inputs:context'),
                ('Odom.outputs:execOut', 'PubOdom.inputs:execIn'),
                ('Odom.outputs:execOut', 'PubTF.inputs:execIn'),
                ('Odom.outputs:position', 'PubOdom.inputs:position'),
                ('Odom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('Odom.outputs:linearVelocity', 'PubOdom.inputs:linearVelocity'),
                ('Odom.outputs:angularVelocity', 'PubOdom.inputs:angularVelocity'),
                ('Odom.outputs:position', 'PubTF.inputs:translation'),
                ('Odom.outputs:orientation', 'PubTF.inputs:rotation'),
                ('Context.outputs:context', 'PubOdom.inputs:context'),
                ('Context.outputs:context', 'PubTF.inputs:context'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
                ('SimTime.outputs:simulationTime', 'PubTF.inputs:timeStamp'),
            ],
        },
    )


class KinematicBase:

    def __init__(self, articulation):
        self.robot = Articulation(articulation)
        self.lin = og.Controller.attribute(CMD_VEL_NODE + '.outputs:linearVelocity')
        self.ang = og.Controller.attribute(CMD_VEL_NODE + '.outputs:angularVelocity')
        self.v = self.w = 0.0

    @staticmethod
    def _ramp(current, target, limit):
        step = limit * PHYSICS_DT
        return current + max(-step, min(step, target - current))

    def step(self):
        self.v = self._ramp(self.v, float(self.lin.get()[0]), MAX_ACCELERATION)
        self.w = self._ramp(self.w, float(self.ang.get()[2]), MAX_ANGULAR_ACCELERATION)
        positions, orientations = self.robot.get_world_poses()
        x, y, _ = positions.numpy()[0]
        qw, qx, qy, qz = orientations.numpy()[0]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
        self.robot.set_world_poses([[x, y, BASE_HEIGHT]],
                                   [[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]])
        self.robot.set_velocities([[self.v * math.cos(yaw), self.v * math.sin(yaw), 0.0]],
                                  [[0.0, 0.0, self.w]])


def main():
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())

    build_world(stage, args.world)

    usd_file, lidar_xyz = import_robot()
    spawn = UsdGeom.Xform.Define(stage, '/World/shoddy')
    UsdGeom.XformCommonAPI(spawn).SetTranslate((args.x, args.y, BASE_HEIGHT))
    UsdGeom.XformCommonAPI(spawn).SetRotate((0.0, 0.0, math.degrees(args.yaw)))
    robot = stage.DefinePrim('/World/shoddy/robot')
    robot.GetReferences().AddReference(usd_file)
    robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    make_frictionless(stage, robot)
    simulation_app.update()
    articulation, base_link = find_prims(robot)
    print(f'[robocup_isaac] articulation={articulation} base_link={base_link} lidar_xyz={lidar_xyz}')

    lidar_path = base_link + '/rtx_lidar'
    Lidar.create(path=lidar_path, config='Example_Rotary_2D', tick_rate=10.0,
                 translations=[lidar_xyz],
                 attributes={'omni:sensor:Core:nearRangeM': 0.15,
                             'omni:sensor:Core:farRangeM': 4.0})

    SimulationManager.setup_simulation(dt=PHYSICS_DT, device='cpu')
    simulation_app.update()

    create_ros2_clock_graph(Ros2ClockGraphConfig(graph_path='/Graph/ROS_Clock'))
    create_base_graph(articulation)
    create_ros2_rtx_lidar_graph(Ros2RtxLidarGraphConfig(
        graph_path='/Graph/ROS_Lidar', lidar_prim=lidar_path,
        frame_id='lidar_frame', laser_scan_topic='/scan_raw'))
    simulation_app.update()

    app_utils.play()
    simulation_app.update()
    base = KinematicBase(articulation)
    SimulationManager.register_callback(lambda dt, context: base.step(),
                                        event=SimulationEvent.PHYSICS_PRE_STEP)
    print('[robocup_isaac] simulation running', flush=True)
    frame = 0
    while simulation_app.is_running():
        simulation_app.update()
        frame += 1
        if args.frames and frame >= args.frames:
            break
    app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
finally:
    simulation_app.close()
