import argparse
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on interacting with a particle grid.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.utils.math as math_utils
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.prims.soft.particle_system_view import ParticleSystemView
from omni.isaac.lab.sim import SimulationContext
from pxr import Sdf, Gf, Vt, UsdGeom
import omni.physx.scripts.particleUtils as particleUtils
from omni.isaac.core.utils.stage import get_current_stage


def design_scene():
    """Designs a simple scene with a particle grid."""
    # Ground-plane
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    
    # Lights
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    light_cfg.func("/World/Light", light_cfg)

    # Create origin for the particle grid
    origin = [0.25, 0.25, 0.0]
    prim_utils.create_prim("/World/Origin0", "Xform", translation=origin)

    # Particle System Configuration
    particle_system_path = "/World/Origin0/ParticleSystem"
    particle_system = ParticleSystem(
        prim_path=particle_system_path,
        particle_system_enabled=True,
        simulation_owner="/World/PhysicsScene",
        rest_offset=0.005,
        contact_offset=0.015,
        solid_rest_offset=0.005,
        particle_contact_offset=0.015,
        max_velocity=100.0,
        max_neighborhood=100
    )

    # Create a grid of particles
    particle_spacing = 0.02
    grid_size = (10, 10, 10)  # Define grid size (X, Y, Z dimensions)
    grid_origin = Gf.Vec3f(0.0, 0.0, 0.5)

    positions, velocities = particleUtils.create_particles_grid(
        lower=grid_origin, 
        particle_spacing=particle_spacing,
        dim_x=grid_size[0], 
        dim_y=grid_size[1], 
        dim_z=grid_size[2]
    )

    # particle_point_instancer_path = particle_system_path + "/particles"
    particle_point_instancer_path = Sdf.Path(particle_system_path + "/particles")

    # stage = get_current_stage()
    particleUtils.add_physx_particleset_pointinstancer(
        stage=get_current_stage(),
        path=particle_point_instancer_path,
        positions=Vt.Vec3fArray(positions),
        velocities=Vt.Vec3fArray(velocities),
        particle_system_path=particle_system_path,
        self_collision=True,
        fluid=False,
        particle_group=0,
        particle_mass=0.0,
        density = 0.0,
    )

    return {"particle_system": particle_system}


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, ParticleSystem]):
    """Runs the simulation loop."""
    particle_system = entities["particle_system"]
    sim_dt = sim.get_physics_dt()

    while simulation_app.is_running():
        # Write internal data to simulation
        # scene.add(particle_system)
        # Perform simulation step
        sim.step()
        # Update buffers
        # particle_system.update(sim_dt)


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[3.0, 0.0, 1.0], target=[0.0, 0.0, 0.5])
    
    scene_entities = design_scene()
    sim.reset()
    print("[INFO]: Setup complete...")

    run_simulator(sim, scene_entities)


if __name__ == "__main__":
    main()
    simulation_app.close()
