import creature
import population
import simulation
import genome
import numpy as np

# Adjust basic variables for basic coursework
num_iterations = 501
pop_size = 50
gene_count = 3
sim_time = 4800

tournament_size = int(pop_size * 0.1)
point_mutate = 0.2
shrink_mutate = 0.25
grow_mutate = 0.10

pop = population.Population(pop_size=pop_size, gene_count=gene_count)
sim = simulation.Simulation(sim_time=sim_time)

# Create csv_stats file to record stats of genetic algorithm.
csv_stats_file = open('stats.csv', 'x')
csv_stats_file.write('iteration, fittest_fit, average_fit, fittest_mountain_fitness, average_mountain_fitness, '
                     'fittest_mountain_touching_fitness, average_mountain_touching_fitness, fittest_size_fitness, '
                     'average_size_fitness, fittest_distance, mean_distance, fittest_vertical_distance, '
                     'average_vertical_distance, max_links, average_links\n')
csv_stats_file.close()

for iteration in range(num_iterations):
    for cr in pop.creatures:
        sim.run_creature(cr)

    fits = [cr.get_fitness(sim_time) for cr in pop.creatures]
    links = [len(cr.get_expanded_links()) for cr in pop.creatures]

    mountain_top_fitness = [cr.distance_fitness_helper() for cr in pop.creatures]
    touching_mountain_fitness = [cr.touching_mountain_fitness_helper(sim_time) for cr in pop.creatures]
    size_fitness = [cr.size_fitness_helper() for cr in pop.creatures]
    distance_fitness = [cr.distance_travelled_fitness_helper() for cr in pop.creatures]
    vertical_distance_fitness = [cr.vertical_distance_fitnees_helper() for cr in pop.creatures]

    # Write current population stats to stats.csv
    csv_stats_file = open('stats.csv', 'a')
    stats_line = f'{iteration}, ' \
                 f'{np.round(np.max(fits), 3)}, ' \
                 f'{np.round(np.mean(fits), 3)}, ' \
                 f'{np.round(np.max(mountain_top_fitness), 3)}, ' \
                 f'{np.round(np.mean(mountain_top_fitness), 3)}, ' \
                 f'{np.round(np.max(touching_mountain_fitness), 3)}, ' \
                 f'{np.round(np.mean(touching_mountain_fitness), 3)}, ' \
                 f'{np.round(np.min(size_fitness), 3)}, ' \
                 f'{np.round(np.mean(size_fitness), 3)}, ' \
                 f'{np.round(np.max(distance_fitness), 3)}, ' \
                 f'{np.round(np.mean(distance_fitness), 3)}, ' \
                 f'{np.round(np.max(vertical_distance_fitness), 3)}, ' \
                 f'{np.round(np.mean(vertical_distance_fitness), 3)}, ' \
                 f'{np.round(np.max(links))}, ' \
                 f'{np.round(np.mean(links))}\n'
    csv_stats_file.write(stats_line)
    csv_stats_file.close()

    new_creatures = []

    for i in range(len(pop.creatures)):
        p1_ind = population.Population.tournament_fitness(fits, tournament_size)
        p2_ind = population.Population.tournament_fitness(fits, tournament_size)
        p1 = pop.creatures[p1_ind]
        p2 = pop.creatures[p2_ind]

        # now we have the parents!
        dna = genome.Genome.crossover(p1.dna, p2.dna)
        dna = genome.Genome.point_mutate(dna, rate=point_mutate)
        dna = genome.Genome.shrink_mutate(dna, rate=shrink_mutate)
        dna = genome.Genome.grow_mutate(dna, rate=grow_mutate)
        cr = creature.Creature(1)
        cr.update_dna(dna)
        new_creatures.append(cr)

    # elitism
    max_fit = np.max(fits)

    # Limit production of elite creatures with large iterations.
    if iteration % 10 == 0:
        # Get elite creature for given population.
        for cr in pop.creatures:
            if cr.get_fitness(sim_time) == max_fit:
                new_cr = creature.Creature(1)
                new_cr.update_dna(cr.dna)
                new_creatures[0] = new_cr
                filename = "elite_" + str(iteration) + ".csv"
                genome.Genome.to_csv(cr.dna, filename)
                break

    # Make the new population the current population.
    pop.creatures = new_creatures

    # Indicate iteration is complete
    print('Iteration', iteration, 'complete.')
