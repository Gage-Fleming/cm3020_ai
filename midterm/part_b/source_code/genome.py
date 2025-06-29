import numpy as np
import copy
import random


class Genome:
    @staticmethod
    def get_random_gene(length):
        gene = np.array([np.random.random() for _ in range(length)])
        return gene

    @staticmethod
    def get_random_genome(gene_length, gene_count):
        genome = [Genome.get_random_gene(gene_length) for _ in range(gene_count)]
        return genome

    # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
    # Add evolve values to genes.
    @staticmethod
    def get_gene_spec():
        gene_spec = {
            "link-shape": {"scale": 1, "evolve": True},
            "link-length": {"scale": 2, "evolve": True},
            "link-radius": {"scale": 1, "evolve": True},
            "link-recurrence": {"scale": 3, "evolve": True},
            "link-mass": {"scale": 1, "evolve": True},
            "joint-type": {"scale": 1, "evolve": True},
            "joint-parent": {"scale": 1, "evolve": True},
            "joint-axis-xyz": {"scale": 1, "evolve": True},
            "joint-origin-rpy-1": {"scale": np.pi * 2, "evolve": False},
            "joint-origin-rpy-2": {"scale": np.pi * 2, "evolve": False},
            "joint-origin-rpy-3": {"scale": np.pi * 2, "evolve": False},
            "joint-origin-xyz-1": {"scale": 1, "evolve": False},
            "joint-origin-xyz-2": {"scale": 1, "evolve": False},
            "joint-origin-xyz-3": {"scale": 1, "evolve": False},
            "control-waveform": {"scale": 1, "evolve": True},
            "control-amp": {"scale": 0.25, "evolve": True},
            "control-freq": {"scale": 1, "evolve": True}
        }
        # *** END OF CODE PARTIALLY WRITTEN BY GAGE FLEMING ***

        ind = 0

        for key in gene_spec.keys():
            gene_spec[key]["ind"] = ind
            ind = ind + 1

        return gene_spec

    @staticmethod
    def get_gene_dict(gene, spec):
        gdict = {}

        for key in spec:
            ind = spec[key]["ind"]
            scale = spec[key]["scale"]
            gdict[key] = gene[ind] * scale

        return gdict

    @staticmethod
    def get_genome_dicts(genome, spec):
        g_dicts = []

        for gene in genome:
            g_dicts.append(Genome.get_gene_dict(gene, spec))

        return g_dicts

    @staticmethod
    def expand_links(parent_link, uniq_parent_name, flat_links, exp_links):
        children = [link for link in flat_links if link.parent_name == parent_link.name]
        sibling_ind = 1

        for c in children:
            for r in range(int(c.recur)):
                sibling_ind = sibling_ind + 1
                c_copy = copy.copy(c)
                c_copy.parent_name = uniq_parent_name
                uniq_name = c_copy.name + str(len(exp_links))
                c_copy.name = uniq_name
                c_copy.sibling_ind = sibling_ind
                exp_links.append(c_copy)
                assert c.parent_name != c.name, ("Genome::expandLinks: link joined to itself: "
                                                 + c.name + " joins " + c.parent_name)
                Genome.expand_links(c, uniq_name, flat_links, exp_links)

    @staticmethod
    def genome_to_links(g_dicts):
        links = []
        link_ind = 0
        parent_names = [str(link_ind)]

        for gdict in g_dicts:
            link_name = str(link_ind)
            # *** CODE WRITTEN BY GAGE FLEMING ***
            # Change how parent value is calculated to avoid underflow error.
            parent_ind = int(gdict["joint-parent"] * (len(parent_names) - 1))
            parent_ind = max(0, min(parent_ind, len(parent_names) - 1))
            # *** END OF CODE WRITTEN BY GAGE FLEMING ***
            parent_name = parent_names[int(parent_ind)]
            recur = gdict["link-recurrence"]
            link = URDFLink(name=link_name,
                            parent_name=parent_name,
                            recur=recur + 1,
                            link_length=gdict["link-length"],
                            link_radius=gdict["link-radius"],
                            link_mass=gdict["link-mass"],
                            joint_type=gdict["joint-type"],
                            joint_parent=gdict["joint-parent"],
                            joint_axis_xyz=gdict["joint-axis-xyz"],
                            joint_origin_rpy_1=gdict["joint-origin-rpy-1"],
                            joint_origin_rpy_2=gdict["joint-origin-rpy-2"],
                            joint_origin_rpy_3=gdict["joint-origin-rpy-3"],
                            joint_origin_xyz_1=gdict["joint-origin-xyz-1"],
                            joint_origin_xyz_2=gdict["joint-origin-xyz-2"],
                            joint_origin_xyz_3=gdict["joint-origin-xyz-3"],
                            control_waveform=gdict["control-waveform"],
                            control_amp=gdict["control-amp"],
                            control_freq=gdict["control-freq"])
            links.append(link)

            if link_ind != 0:  # don't re-add the first link
                parent_names.append(link_name)
            link_ind = link_ind + 1

        # now just fix the first link, so it links to nothing
        links[0].parent_name = "None"

        return links

    @staticmethod
    def crossover(g1, g2):
        x1 = random.randint(0, len(g1) - 1)
        x2 = random.randint(0, len(g2) - 1)
        g3 = np.concatenate((g1[x1:], g2[x2:]))

        if len(g3) > len(g1):
            g3 = g3[0:len(g1)]

        return g3

    # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
    # Add gene spec parameter to mutation functions to help with evolution checker.
    @staticmethod
    def point_mutate(genome, rate, gene_spec):
        new_genome = copy.copy(genome)

        # Add check to see if gene is mutable.
        for gene in new_genome:
            # Iterate through gene spec.
            for i, (key, spec) in enumerate(gene_spec.items()):
                # If gene can evolve and the random rate is above check mutate.
                if spec["evolve"] and random.random() < rate:
                    gene[i] += random.uniform(-0.1, 0.1)
                    # Ensure gene is not negative.
                    gene[i] = max(0, min(gene[i], 1))

        return new_genome

    @staticmethod
    def shrink_mutate(genome, rate, gene_spec):
        if len(genome) == 1:
            return copy.copy(genome)
        # Add check to see if gene is mutable.
        if random.random() < rate:
            # Get the gene's that are mutable.
            evolve_indices = [i for i, gene in enumerate(genome) if
                              any(gene_spec[key]["evolve"] for key in gene_spec)]
            # If evolve genes exist they are candidates for shrink mutate.
            if evolve_indices:
                ind = random.choice(evolve_indices)
                new_genome = np.delete(genome, ind, 0)
                return new_genome
        else:
            return copy.copy(genome)

    @staticmethod
    def grow_mutate(genome, rate, gene_spec):
        # Add check to see if gene is mutable.
        if random.random() < rate:
            # Create new gene.
            new_gene = Genome.get_random_gene(len(genome[0]))
            # Iterate through gene spec.
            for i, (key, spec) in enumerate(gene_spec.items()):
                # Check if gene can be evolved.
                if not spec["evolve"]:
                    new_gene[i] = genome[0][i]
            new_genome = np.append(genome, [new_gene], axis=0)
            return new_genome
        else:
            return copy.copy(genome)

    # *** END OF CODE PARTIALLY WRITTEN BY GAGE FLEMING ***

    @staticmethod
    def to_csv(dna, csv_file):
        csv_str = ""
        for gene in dna:
            for val in gene:
                csv_str = csv_str + str(val) + ","

            csv_str = csv_str + '\n'

        with open(csv_file, 'w') as f:
            f.write(csv_str)

    @staticmethod
    def from_csv(filename):
        csv_str = ''
        with open(filename) as f:
            csv_str = f.read()
        dna = []
        lines = csv_str.split('\n')
        for line in lines:
            vals = line.split(',')
            gene = [float(v) for v in vals if v != '']
            if len(gene) > 0:
                dna.append(gene)
        return dna


class URDFLink:
    def __init__(self, name, parent_name, recur,
                 link_length=0.1,
                 link_radius=0.1,
                 link_mass=0.1,
                 joint_type=0.1,
                 joint_parent=0.1,
                 joint_axis_xyz=0.1,
                 joint_origin_rpy_1=0.1,
                 joint_origin_rpy_2=0.1,
                 joint_origin_rpy_3=0.1,
                 joint_origin_xyz_1=0.1,
                 joint_origin_xyz_2=0.1,
                 joint_origin_xyz_3=0.1,
                 control_waveform=0.1,
                 control_amp=0.1,
                 control_freq=0.1,
                 link_shape=0.25):
        self.name = name
        self.parent_name = parent_name
        self.recur = recur
        self.link_length = link_length
        self.link_radius = link_radius
        self.link_mass = link_mass
        self.joint_type = joint_type
        self.joint_parent = joint_parent
        self.joint_axis_xyz = joint_axis_xyz
        self.joint_origin_rpy_1 = joint_origin_rpy_1
        self.joint_origin_rpy_2 = joint_origin_rpy_2
        self.joint_origin_rpy_3 = joint_origin_rpy_3
        self.joint_origin_xyz_1 = joint_origin_xyz_1
        self.joint_origin_xyz_2 = joint_origin_xyz_2
        self.joint_origin_xyz_3 = joint_origin_xyz_3
        self.control_waveform = control_waveform
        self.control_amp = control_amp
        self.control_freq = control_freq
        self.sibling_ind = 1

        self.link_shape = link_shape

    def to_link_element(self, adom):
        link_tag = adom.createElement("link")
        link_tag.setAttribute("name", self.name)
        vis_tag = adom.createElement("visual")
        geom_tag = adom.createElement("geometry")

        # Add more potential shapes. Shapes and code referenced from reference [4].
        if self.link_shape <= 0.25:
            # Link will be a cylinder.
            shape_tag = adom.createElement("cylinder")
            # Store shape of cylinder.
            shape_tag.setAttribute("length", str(self.link_length))
            shape_tag.setAttribute("radius", str(self.link_radius))
        elif self.link_shape <= 0.5:
            # Link will be a box.
            shape_tag = adom.createElement("box")
            # Store shape of box.
            shape_tag.setAttribute("size", f"{self.link_radius * 2} {self.link_radius * 2} {self.link_length}")
        elif self.link_shape <= 0.75:
            # Link will be a sphere.
            shape_tag = adom.createElement("sphere")
            # Store shape of sphere.
            shape_tag.setAttribute("radius", str(self.link_radius))
        else:
            # Link will be a capsule.
            shape_tag = adom.createElement("capsule")
            # Store shape of capsule.
            shape_tag.setAttribute("length", str(self.link_length))
            shape_tag.setAttribute("radius", str(self.link_radius))

        geom_tag.appendChild(shape_tag)
        vis_tag.appendChild(geom_tag)

        coll_tag = adom.createElement("collision")
        c_geom_tag = adom.createElement("geometry")
        c_shape_tag = shape_tag.cloneNode(True)

        c_geom_tag.appendChild(c_shape_tag)
        coll_tag.appendChild(c_geom_tag)

        inertial_tag = adom.createElement("inertial")
        mass_tag = adom.createElement("mass")

        mass = np.pi * (self.link_radius * self.link_radius) * self.link_length
        mass_tag.setAttribute("value", str(mass))
        inertia_tag = adom.createElement("inertia")

        inertia_tag.setAttribute("ixx", "0.03")
        inertia_tag.setAttribute("iyy", "0.03")
        inertia_tag.setAttribute("izz", "0.03")
        inertia_tag.setAttribute("ixy", "0")
        inertia_tag.setAttribute("ixz", "0")
        inertia_tag.setAttribute("iyx", "0")
        inertial_tag.appendChild(mass_tag)
        inertial_tag.appendChild(inertia_tag)

        link_tag.appendChild(vis_tag)
        link_tag.appendChild(coll_tag)
        link_tag.appendChild(inertial_tag)

        return link_tag

    def to_joint_element(self, adom):
        joint_tag = adom.createElement("joint")
        joint_tag.setAttribute("name", self.name + "_to_" + self.parent_name)
        if self.joint_type >= 0.5:
            joint_tag.setAttribute("type", "revolute")
        else:
            joint_tag.setAttribute("type", "revolute")
        parent_tag = adom.createElement("parent")
        parent_tag.setAttribute("link", self.parent_name)
        child_tag = adom.createElement("child")
        child_tag.setAttribute("link", self.name)
        axis_tag = adom.createElement("axis")

        if self.joint_axis_xyz <= 0.33:
            axis_tag.setAttribute("xyz", "1 0 0")
        if self.joint_axis_xyz > 0.33 and self.joint_axis_xyz <= 0.66:
            axis_tag.setAttribute("xyz", "0 1 0")
        if self.joint_axis_xyz > 0.66:
            axis_tag.setAttribute("xyz", "0 0 1")

        limit_tag = adom.createElement("limit")

        limit_tag.setAttribute("effort", "1")
        limit_tag.setAttribute("upper", "-3.1415")
        limit_tag.setAttribute("lower", "3.1415")
        limit_tag.setAttribute("velocity", "1")

        orig_tag = adom.createElement("origin")

        rpy1 = self.joint_origin_rpy_1 * self.sibling_ind
        rpy = str(rpy1) + " " + str(self.joint_origin_rpy_2) + " " + str(self.joint_origin_rpy_3)

        orig_tag.setAttribute("rpy", rpy)
        xyz = str(self.joint_origin_xyz_1) + " " + str(self.joint_origin_xyz_2) + " " + str(self.joint_origin_xyz_3)
        orig_tag.setAttribute("xyz", xyz)

        joint_tag.appendChild(parent_tag)
        joint_tag.appendChild(child_tag)
        joint_tag.appendChild(axis_tag)
        joint_tag.appendChild(limit_tag)
        joint_tag.appendChild(orig_tag)

        return joint_tag
