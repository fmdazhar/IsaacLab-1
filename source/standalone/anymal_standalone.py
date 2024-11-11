# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.quadruped.robots import AnymalFlatTerrainPolicy
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.simulation_context import SimulationContext

from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Vt, UsdLux, UsdShade
from omni.physx.scripts import physicsUtils, particleUtils
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.prims.soft.particle_system_view import ParticleSystemView
import random
from omni.isaac.core.prims import XFormPrim




class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        """
        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        self.stage = get_current_stage()

        simulation_context = SimulationContext.instance()
        simulation_context.get_physics_context().enable_gpu_dynamics(True)
        simulation_context.get_physics_context().set_broadphase_type("GPU")

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # # spawn ground scene
        # self._world.scene.add_default_ground_plane(
        #     z_position=0,
        #     name="default_ground_plane",
        #     prim_path="/World/defaultGroundPlane",
        #     static_friction=0.2,
        #     dynamic_friction=0.2,
        #     restitution=0.01,
        # )

        # Create custom terrain
        self.generate_central_depression_terrain()

        self._anymal = AnymalFlatTerrainPolicy(
            prim_path="/World/Anymal",
            name="Anymal",
            usd_path=assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c.usd",
            position=np.array([0, 0, 0.7]),
        )

        light = UsdLux.DistantLight.Define(self.stage, "/World/defaultDistantLight")
        light.CreateIntensityAttr().Set(5000)

              
        self._base_command = np.zeros(3)

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.0, 0.0, 0.0],
            "UP": [1.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.0, 0.0],
            "RIGHT": [0.0, -1.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.0, 0.0],
            "LEFT": [0.0, 1.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],
        }
        self.needs_reset = False
        self.first_step = True

    def generate_central_depression_terrain(self):
        """
        Generate a terrain with a central negative heightfield depression
        that matches the particle grid's position and size.
        """
        # Terrain parameters
        width = 256          # Number of grid points along the width
        length = 256         # Number of grid points along the length
        vertical_scale = 0.05  # Meters per heightfield unit
        horizontal_scale = 0.05  # Meters per pixel
        platform_height = 0.0    # Height of the surrounding platform in meters

        # Set depression depth to match particle grid's z-range
        depression_depth = -0.15  # Depth of the depression in meters (negative value)

        # Particle grid parameters (from create_particle_grid method)
        x_position, y_position = 2.5, 0.0  # Center position of particle grid
        scale_x, scale_y = 3.1, 3.1        # Size of the particle grid

        # **Change 1: Set terrain origin to match particle grid center**
        terrain_origin_x = x_position - (width * horizontal_scale) / 2
        terrain_origin_y = y_position - (length * horizontal_scale) / 2

        # Convert parameters to discrete units
        depression_depth_units = int(depression_depth / vertical_scale)
        platform_height_units = int(platform_height / vertical_scale)

        # Create heightfield array
        height_field_raw = np.zeros((width, length), dtype=np.int16)
        height_field_raw[:, :] = platform_height_units

        # **Change 2: Position depression at the center of the terrain**
        center_x = width // 2
        center_y = length // 2
        half_size_x = int((scale_x / 2) / horizontal_scale)
        half_size_y = int((scale_y / 2) / horizontal_scale)

        depression_start_x = center_x - half_size_x
        depression_end_x = center_x + half_size_x
        depression_start_y = center_y - half_size_y
        depression_end_y = center_y + half_size_y

        # Ensure indices are within bounds
        depression_start_x = max(0, depression_start_x)
        depression_end_x = min(width, depression_end_x)
        depression_start_y = max(0, depression_start_y)
        depression_end_y = min(length, depression_end_y)

        # Set heights for depression
        height_field_raw[depression_start_x:depression_end_x, depression_start_y:depression_end_y] = depression_depth_units

        # Convert heightfield to triangle mesh
        num_rows, num_cols = height_field_raw.shape
        y = np.linspace(0, (num_cols - 1) * horizontal_scale, num_cols)
        x = np.linspace(0, (num_rows - 1) * horizontal_scale, num_rows)
        yy, xx = np.meshgrid(y, x)

        # Create vertices
        vertices = np.zeros((num_rows * num_cols, 3), dtype=np.float32)
        vertices[:, 0] = xx.flatten()
        vertices[:, 1] = yy.flatten()
        vertices[:, 2] = height_field_raw.flatten() * vertical_scale

        # Create triangles
        triangles = -np.ones((2 * (num_rows - 1) * (num_cols - 1), 3), dtype=np.uint32)
        for i in range(num_rows - 1):
            ind0 = np.arange(0, num_cols - 1) + i * num_cols
            ind1 = ind0 + 1
            ind2 = ind0 + num_cols
            ind3 = ind2 + 1
            start = 2 * i * (num_cols - 1)
            stop = start + 2 * (num_cols - 1)
            triangles[start:stop:2, 0] = ind0
            triangles[start:stop:2, 1] = ind3
            triangles[start:stop:2, 2] = ind1
            triangles[start + 1 : stop : 2, 0] = ind0
            triangles[start + 1 : stop : 2, 1] = ind2
            triangles[start + 1 : stop : 2, 2] = ind3

        # Add terrain to stage
        num_faces = triangles.shape[0]
        terrain_mesh = self.stage.DefinePrim("/World/terrain", "Mesh")
        terrain_mesh.GetAttribute("points").Set(vertices)
        terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
        terrain_mesh.GetAttribute("faceVertexCounts").Set(np.asarray([3] * num_faces))

        # **Change 1 (continued): Update terrain position to match particle grid**
        terrain = XFormPrim(prim_path="/World/terrain", name="terrain", position=[terrain_origin_x, terrain_origin_y, 0.0])

        # Apply collision properties
        collision_api = UsdPhysics.CollisionAPI.Apply(terrain.prim)
        physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(terrain.prim)
        physx_collision_api.GetContactOffsetAttr().Set(0.02)
        physx_collision_api.GetRestOffsetAttr().Set(0.00)



    def create_particle_system(self):

        # Define paths
        default_prim_path = "/World"
        particle_system_path = default_prim_path + "/particleSystem"

        # Set particle system parameters
        rest_offset = 0.02
        solid_rest_offset = 0.015
        particle_contact_offset = 0.04

        # Create the particle system
        self._particle_system = ParticleSystem(
            prim_path=particle_system_path,
            particle_system_enabled=True,
            simulation_owner="/physicsScene",
            rest_offset=rest_offset,
            contact_offset=particle_contact_offset,
            solid_rest_offset=solid_rest_offset,
            particle_contact_offset=particle_contact_offset,
            max_velocity=100.0,
            max_neighborhood=340,
            solver_position_iteration_count=64,
            # enable_ccd=True,
            # max_depenetration_velocity=1.0,
        )

        # Create the particle prototype
        self.create_pbd_material()

        # Create a grid of particles
        self.create_particle_grid()

    def create_particle_grid(self):
        # Define paths
        default_prim_path = "/World"
        particle_system_path = default_prim_path + "/particleSystem"

        # Define the position and size of the particle grid
        x_position, y_position, z_position = 2.5, 0.0, 0.35  # Adjust as needed
        scale_x, scale_y, scale_z = 3.0, 3.0, 0.3  # Size of the grid
        lower = Gf.Vec3f(x_position-scale_x * 0.5, y_position-scale_y * 0.5, 0)

        solid_rest_offset = 0.015
        particle_spacing = 2.5 * solid_rest_offset

        num_samples_x = int(scale_x / particle_spacing) + 1
        num_samples_y = int(scale_y / particle_spacing) + 1
        num_samples_z = int(scale_z / particle_spacing) + 1

        jitter_factor = 0.2 * particle_spacing  # Adjust the jitter factor as needed

        position = [Gf.Vec3f(0.0)] * num_samples_x * num_samples_y * num_samples_z
        uniform_particle_velocity=Gf.Vec3f(0.0)
        ind = 0
        x = lower[0]
        y = lower[1]
        z = lower[2]
        for i in range(num_samples_x):
            for j in range(num_samples_y):
                for k in range(num_samples_z):
                    jitter_x = random.uniform(-jitter_factor, jitter_factor)
                    jitter_y = random.uniform(-jitter_factor, jitter_factor)
                    jitter_z = random.uniform(-jitter_factor, jitter_factor)
                    
                    # Apply jitter to the position
                    jittered_x = x + jitter_x
                    jittered_y = y + jitter_y
                    jittered_z = z + jitter_z
                    position[ind] = Gf.Vec3f(jittered_x, jittered_y, jittered_z)
                    ind += 1
                    z = z + particle_spacing
                z = lower[2]
                y = y + particle_spacing
            y = lower[1]
            x = x + particle_spacing
        positions, velocities =  (position, [uniform_particle_velocity] * len(position))
        widths = [2 * solid_rest_offset * 0.5] * len(position)


        # Define particle point instancer path
        particle_point_instancer_path = Sdf.Path(particle_system_path + "/particles")

        # Add the particle set to the point instancer
        particleUtils.add_physx_particleset_pointinstancer(
            self.stage,
            particle_point_instancer_path,
            Vt.Vec3fArray(positions),
            Vt.Vec3fArray(velocities),
            particle_system_path,
            self_collision=True,
            fluid=False,
            particle_group=0,
            particle_mass=0.0,
            density=0.0
        )

        # Configure particle prototype
        particle_prototype_sphere = UsdGeom.Sphere.Get(
            self.stage, particle_point_instancer_path.AppendChild("particlePrototype0")
        )
        particle_prototype_sphere.CreateRadiusAttr().Set(solid_rest_offset)


    def create_pbd_material(self):
        ps = PhysxSchema.PhysxParticleSystem.Get(self.stage, Sdf.Path("/World/particleSystem"))
        #setting up a material density, will be used by both ref & cand because of shared particle system
        pbd_material_path = Sdf.Path("/World/pbdmaterial")
        particleUtils.add_pbd_particle_material(self.stage, pbd_material_path, 
        friction = 0.8,  # A middle-ground friction value
        particle_friction_scale=0.2,  # Default scale, not altering the base friction
        # damping=10,  # A low damping to allow for some energy dissipation
        adhesion=0.00001,  # Minimal adhesion to surfaces
        particle_adhesion_scale=50000.0,  # Default scale for particle to particle adhesion
        adhesion_offset_scale=1.2,  # Scales the effect distance for adhesion
        density=1500,  # Density of water in kg/m^3 for fluids, adjust based on your simulation needs
        )
        physicsUtils.add_physics_material_to_prim(self.stage, ps.GetPrim(), pbd_material_path)
       

    def setup(self) -> None:
        """
        Set up keyboard listener and add physics callback

        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self.create_particle_system()  # Create the particle system
        self._world.add_physics_callback("anymal_advance", callback_fn=self.on_physics_step)
        

    def on_physics_step(self, step_size) -> None:
        """
        Physics call back, initialize robot (first frame) and call robot advance function to compute and apply joint torque

        """
        if self.first_step:
            self._anymal.initialize()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._anymal.advance(step_size, self._base_command)

    def run(self) -> None:
        """
        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback to when kit is updated.

        """

        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True


def main():
    """
    Parse arguments and instantiate the ANYmal runner

    """
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner._world.reset()
    runner._world.reset()
    runner._world.reset()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
