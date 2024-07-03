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

    @staticmethod
    def tournament_fitness(fits, size=6):
        creature_indexes = random.sample(range(len(fits)), size)
        creature_fits = [fits[i] for i in creature_indexes]
        winner_index = creature_indexes[np.argmax(creature_fits)]
        return winner_index
