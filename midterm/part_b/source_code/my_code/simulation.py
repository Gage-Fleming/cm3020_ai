import pybullet as p
import pybullet_data
import environment_helper as eh
import creature


class Simulation:
    def __init__(self, sim_id=0, sim_time=2400):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id
        self.sim_time = sim_time

    def run_creature(self, cr):
        pid = self.physicsClientId
        p.resetSimulation()
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Get top of mountain for fitness function in x,y,z
        mountain_height, mid = eh.make_landscape()
        top_of_mountain = (0, 0, mountain_height)

        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)

        cid = p.loadURDF(xml_file)

        p.resetBasePositionAndOrientation(cid, [5, 5, 1], [0, 0, 0, 1])

        for step in range(self.sim_time):
            p.stepSimulation()
            if step % 24 == 0:
                self.update_motors(cid=cid, cr=cr)

            pos, orn = p.getBasePositionAndOrientation(cid)
            cr.update_position(pos)

            cr.update_closest_distance_from_mountain_top(mountain_top=top_of_mountain)
            cr.check_if_creature_touching_Mountain(cid=cid, mid=mid)

    def update_motors(self, cid, cr):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]
            p.setJointMotorControl2(cid, jid,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=m.get_output(),
                                    force=5,
                                    physicsClientId=self.physicsClientId)
