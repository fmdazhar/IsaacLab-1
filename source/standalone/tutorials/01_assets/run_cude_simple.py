import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on interacting with a deformable object.")
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
from omni.isaac.lab.assets import DeformableObject, DeformableObjectCfg
from omni.isaac.lab.sim import SimulationContext


def design_scene():
    """Designs a simple scene."""
    # Ground-plane
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    
    # Lights
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    light_cfg.func("/World/Light", light_cfg)

    # Create origin for the object
    origin = [0.25, 0.25, 0.0]
    prim_utils.create_prim("/World/Origin0", "Xform", translation=origin)

    # Deformable Object Configuration
    deformable_cfg = DeformableObjectCfg(
        prim_path="/World/Origin0/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.2, 0.2, 0.2),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
        debug_vis=True,
    )
    cube_object = DeformableObject(cfg=deformable_cfg)

    return {"cube_object": cube_object}

def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, DeformableObject]):
    """Runs the simulation loop."""
    cube_object = entities["cube_object"]
    sim_dt = sim.get_physics_dt()

    while simulation_app.is_running():
        # Write internal data to simulation
        # cube_object.write_data_to_sim()
        # Perform simulation step
        sim.step()
        # Update buffers
        cube_object.update(sim_dt)

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
