import pybullet as p
import genome
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
from scipy import signal


class MotorType(Enum):
    PULSE = 1
    SINE = 2

    # *** CODE WRITTEN BY GAGE FLEMING ***
    # Add new wavelengths for motor controls.
    TRIANGLE = 3
    SAWTOOTH = 4
    # *** END OF CODE WRITTEN BY GAGE FLEMING ***


class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
        if control_waveform <= 0.25:
            self.motor_type = MotorType.PULSE
        elif control_waveform <= 0.50:
            self.motor_type = MotorType.SINE
        elif control_waveform <= 0.75:
            self.motor_type = MotorType.TRIANGLE
        else:
            self.motor_type = MotorType.SAWTOOTH
        # *** END OF CODE PARTIALLY WRITTEN BY GAGE FLEMING ***

        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)

        # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
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
        # *** END OF CODE PARTIALLY WRITTEN BY GAGE FLEMING ***

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
        # *** CODE WRITTEN BY GAGE FLEMING ***
        # Track the closest distance to mountain.
        self.closest_distance_to_mountain_top = None
        # Track number of times touching mountain.
        self.number_of_times_touching_mountain = 0
        # Store the creature id.
        self.cid = None
        # Store the creature size.
        self.size = None
        # Store if creature failed.
        self.failed = False
        # *** END CODE WRITTEN BY GAGE FLEMING ***

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

    # *** CODE WRITTEN BY GAGE FLEMING ***
    def update_closest_distance_from_mountain_top(self, mountain_top):
        """
        Update the closest distance the creature has been to the mountain top.

        Args:
            mountain_top (tuple): The (x, y, z) coordinates of the mountain top.

        Returns:
            None

        Note:
            If the creature fails to load or encounters an error, it will return 100
            to purposefully make it fail the fitness test.
        """
        # If creature fails to load or encounters error, purposefully make it fail fit test.
        if self.start_position is None or self.last_position is None:
            return 100

        # Get mountain top position and creatures last position.
        p1 = np.asarray(mountain_top)
        p2 = np.asarray(self.last_position)

        # Get the distance between the two.
        dist = np.linalg.norm(p1 - p2)

        # Update if it's closer than previous closest distance.
        if self.closest_distance_to_mountain_top is None or dist < self.closest_distance_to_mountain_top:
            self.closest_distance_to_mountain_top = dist

    def check_if_creature_touching_mountain(self, mid):
        """
        Check if the creature is touching the mountain and update the count if it is.

        Args:
            mid (int): The body ID of the mountain in the physics simulation.

        Returns:
            None
        """

        # Get contact points with mountain [7].
        contact_points = p.getContactPoints(bodyA=self.cid, bodyB=mid)

        # If touching mountain increment tracker.
        if len(contact_points) > 0:
            self.number_of_times_touching_mountain += 1

    def get_closest_distance_to_mountain(self):
        """
        Check if the creature is touching the mountain and update the count if it is.

        Args:
            mid (int): The body ID of the mountain in the physics simulation.

        Returns:
            None
        """
        return self.closest_distance_to_mountain_top

    def get_number_of_times_touching_mountain(self):
        """
        Get the closest distance the creature has been to the mountain top.

        Returns:
            float: The closest distance to the mountain top.
        """
        return self.number_of_times_touching_mountain

    def fail_creature(self):
        """
        Mark the creature as failed.

        Returns:
            None
        """
        self.failed = True

    def set_cid(self, cid):
        """
        Set the creature's ID.

        Args:
            cid (int): The creature's ID in pybullet.

        Returns:
            None
        """
        self.cid = cid

    def set_size(self):
        """
        Calculate and set the size of the creature based on its bounding box.

        Returns:
            None
        """

        # Get dimensions of creature [8].
        aabb_min, aabb_max = p.getAABB(self.cid)
        creature_dimensions = np.array(aabb_max) - np.array(aabb_min)
        # Set the size of the creature.
        self.size = np.max(creature_dimensions)

    def get_size(self):
        """
        Get the size of the creature.

        Returns:
            float: The maximum dimension of the creature's bounding box.
        """
        return self.size

    def get_fitness(self, sim_time):
        """
        Calculate the fitness of the creature based on various factors.

        Args:
            sim_time (float): The total simulation time.

        Returns:
            float: The calculated fitness score, always non-negative.
        """
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
        """
        Calculate the fitness component based on the creature's closest distance to the mountain top.

        Returns:
            float: The distance-based fitness score, ranging from 0 to 100.
        """
        # Fail creature.
        if self.failed:
            return 0

        return min((100 / self.closest_distance_to_mountain_top), 100)

    def size_fitness_helper(self):
        """
        Calculate the fitness penalty based on the creature's size deviation from the optimal size.

        Returns:
            float: The size-based fitness penalty, ranging from 0 to 50.
        """
        # Fail creature if size is not set.
        if self.size is None or self.failed:
            return 50

        # Define optimal size creature should be sticking around.
        optimal_size = 1.0

        # Get the difference from the actual size compared to the optimal size.
        size_difference = abs(self.size - optimal_size)
        return min(50 * (size_difference / optimal_size), 50)

    def touching_mountain_fitness_helper(self, sim_time):
        """
        Calculate the fitness component based on how often the creature touches the mountain.

        Args:
            sim_time (float): The total simulation time.

        Returns:
            float: The mountain-touching fitness score, ranging from 0 to 50.
        """
        # Fail creature.
        if self.failed:
            return 0

        return min((50 * (self.number_of_times_touching_mountain / sim_time)), 50)

    def vertical_distance_fitnees_helper(self):
        """
        Calculate the fitness component based on the creature's vertical progress.

        Returns:
            float: The vertical distance fitness score, ranging from 0 to 50.
        """
        # Fail creature if no position is retrievable.
        if self.start_position is None or self.last_position is None or self.failed:
            return 0

        # Get the vertical progress through simulation.
        vertical_progress = self.last_position[2] - self.start_position[2]

        # Return a ratio of the vertical progress based on 50.
        return 50 * np.tanh(vertical_progress / 5)

    def distance_travelled_fitness_helper(self):
        """
        Calculate the fitness component based on the total distance travelled by the creature.

        Returns:
            float: The distance travelled fitness score, ranging from 0 to 25.
        """
        # Fail creature if no position is retrievable.
        if self.start_position is None or self.last_position is None or self.failed:
            return 0
        return 25 * np.tanh(self.get_distance_travelled() / 10)

# *** END OF CODE WRITTEN BY GAGE FLEMING ***
