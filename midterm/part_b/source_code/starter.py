import time
import pybullet as p


def create_pybullet():
    p.connect(p.GUI)

    # configuration of the engine:
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)
    p.setGravity(0, 0, -10)


def start_simulation():
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240)


create_pybullet()
start_simulation()
