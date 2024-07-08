import pybullet as p
import genome
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
from scipy import signal


class MotorType(Enum):
    PULSE = 1
    SINE = 2
    TRIANGLE = 3
    SAWTOOTH = 4


class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        if control_waveform <= 0.25:
            self.motor_type = MotorType.PULSE
        elif control_waveform <= 0.50:
            self.motor_type = MotorType.SINE
        elif control_waveform <= 0.75:
            self.motor_type = MotorType.TRIANGLE
        else:
            self.motor_type = MotorType.SAWTOOTH

        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)

        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
        elif self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)
        elif self.motor_type == MotorType.TRIANGLE:
            # Code taken from reference [3].
            output = signal.sawtooth(self.phase, width=0.5)
        elif self.motor_type == MotorType.SAWTOOTH:
            # Code taken from reference [3].
            output = signal.sawtooth(self.phase)

        return output


class Creature:
    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
        self.closest_distance_to_mountain_top = None
        self.number_of_times_touching_mountain = 0
        self.cid = None
        self.size = None
        self.failed = False

    def get_flat_links(self):
        if self.flat_links is None:
            g_dicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(g_dicts)
        return self.flat_links

    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links

        exp_links = [self.flat_links[0]]
        genome.Genome.expand_links(self.flat_links[0],
                                   self.flat_links[0].name,
                                   self.flat_links,
                                   exp_links)
        self.exp_links = exp_links
        return self.exp_links

    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:  # skip the root node!
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe")  # choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_expanded_links()
        if self.motors is None:
            motors = []
            for i in range(1, len(self.exp_links)):
                link = self.exp_links[i]
                m = Motor(link.control_waveform, link.control_amp, link.control_freq)
                motors.append(m)
            self.motors = motors
        return self.motors

    def update_position(self, pos):
        if self.start_position is None:
            self.start_position = pos
        else:
            self.last_position = pos

    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1 - p2)
        return dist

    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

    def update_closest_distance_from_mountain_top(self, mountain_top):
        # If creature fails to load or encounters error, purposefully make it fail fit test.
        if self.start_position is None or self.last_position is None:
            return 100

        p1 = np.asarray(mountain_top)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1 - p2)
        if self.closest_distance_to_mountain_top is None or dist < self.closest_distance_to_mountain_top:
            self.closest_distance_to_mountain_top = dist

    def check_if_creature_touching_mountain(self, mid):
        contact_points = p.getContactPoints(bodyA=self.cid, bodyB=mid)
        if len(contact_points) > 0:
            self.number_of_times_touching_mountain += 1

    def get_closest_distance_to_mountain(self):
        return self.closest_distance_to_mountain_top

    def get_number_of_times_touching_mountain(self):
        return self.number_of_times_touching_mountain

    def fail_creature(self):
        self.failed = True

    def set_cid(self, cid):
        self.cid = cid

    def set_size(self):
        aabb_min, aabb_max = p.getAABB(self.cid)
        creature_dimensions = np.array(aabb_max) - np.array(aabb_min)
        self.size = np.max(creature_dimensions)

    def get_size(self):
        return self.size

    def get_fitness(self, sim_time):
        fitness = 0

        # If creature failed at any point return 0.
        if self.failed:
            return fitness

        # Reward for getting closer to mountaintop limited to a max of 100.
        fitness += self.distance_fitness_helper()

        # Reward for vertical progress to a max of 50.
        fitness += self.vertical_distance_fitnees_helper()

        # Reward for distance travelled progress to a max of 50.
        fitness += self.distance_travelled_fitness_helper()

        # Penalty for size of creature to discourage larger creatures to a max of 25.
        fitness -= self.size_fitness_helper()

        # Reward for touching the mountain during simulation to a max of 50.
        fitness += self.touching_mountain_fitness_helper(sim_time)

        return max(fitness, 0)

    def distance_fitness_helper(self):
        # Fail creature.
        if self.failed:
            return 0

        return min((100 / self.closest_distance_to_mountain_top), 100)

    def size_fitness_helper(self):
        # Fail creature if size is not set.
        if self.size is None or self.failed:
            return 50

        # Define optimal size creature should be sticking around.
        optimal_size = 1.0

        # Get the difference from the actual size compared to the optimal size.
        size_difference = abs(self.size - optimal_size)
        return min(50 * (size_difference / optimal_size), 50)

    def touching_mountain_fitness_helper(self, sim_time):
        # Fail creature.
        if self.failed:
            return 0

        return min((50 * (self.number_of_times_touching_mountain / sim_time)), 50)

    def vertical_distance_fitnees_helper(self):
        # Fail creature if no position is retrievable.
        if self.start_position is None or self.last_position is None or self.failed:
            return 0

        # Get the vertical progress through simulation.
        vertical_progress = self.last_position[2] - self.start_position[2]

        # Return a ratio of the vertical progress based on 50.
        return 50 * np.tanh(vertical_progress / 5)

    def distance_travelled_fitness_helper(self):
        # Fail creature if no position is retrievable.
        if self.start_position is None or self.last_position is None or self.failed:
            return 0
        return 25 * np.tanh(self.get_distance_travelled() / 10)
