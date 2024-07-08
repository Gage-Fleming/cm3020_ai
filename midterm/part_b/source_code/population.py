import creature
import numpy as np
import random


class Population:
    def __init__(self, pop_size, gene_count):
        self.creatures = [creature.Creature(gene_count=gene_count) for _ in range(pop_size)]

    @staticmethod
    def get_fitness_map(fits):
        fitmap = []
        total = 0
        for f in fits:
            total = total + f
            fitmap.append(total)

        return fitmap

    @staticmethod
    def select_parent(fitmap):
        r = np.random.rand()  # 0-1
        r = r * fitmap[-1]

        for i in reversed(range(len(fitmap))):
            if r >= fitmap[i]:
                return i

        return 0

    # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
    # Reference 1 guided on implementation.
    @staticmethod
    def tournament_fitness(fits, size=6):
        """
        Perform tournament selection to choose a fit individual.

        Args:
            fits (list): A list of fitness values for all individuals in the population.
            size (int, optional): The number of individuals to include in the tournament. Defaults to 6.

        Returns:
            int: The index of the winner (fittest individual) from the tournament.
        """
        # Get random sample of creatures.
        creature_indexes = random.sample(range(len(fits)), size)
        # Get the creature fits.
        creature_fits = [fits[i] for i in creature_indexes]
        # Choose the highest fit creature.
        winner_index = creature_indexes[np.argmax(creature_fits)]
        return winner_index
    # *** CODE PARTIALLY WRITTEN BY GAGE FLEMING ***
