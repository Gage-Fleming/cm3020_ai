import creature
import population
import simulation
import genome
import numpy as np

# Adjust basic variables for basic coursework
pop_size = 50
gene_count = 5
num_iterations = 101
sim_time = 4800

point_mutate = 0.20
shrink_mutate = 0.1
grow_mutate = 0.1
tournament_size = 3

pop = population.Population(pop_size=pop_size, gene_count=gene_count)
sim = simulation.Simulation(sim_time=sim_time)

# Create csv_stats file to record stats of genetic algorithm.
csv_stats_file = open('stats.csv', 'x')
csv_stats_file.write('fittest, mean, closest_distance_to_mountaintop, average_time_on_mountain, '
                     'smallest_creature_size, largest_creature_size, average_creature_size, mean_links, max_links\n')
csv_stats_file.close()

for iteration in range(num_iterations):
    for cr in pop.creatures:
        sim.run_creature(cr)

    fits = [cr.get_fitness() for cr in pop.creatures]
    links = [len(cr.get_expanded_links()) for cr in pop.creatures]

    mountain_top_distances = [cr.get_closest_distance_to_mountain() for cr in pop.creatures]
    touching_mountain = [cr.get_number_of_times_touching_mountain() for cr in pop.creatures]
    creature_sizes = [cr.get_size() for cr in pop.creatures]

    # Write current population stats to stats.csv
    csv_stats_file = open('stats.csv', 'a')
    stats_line = f'{np.round(np.max(fits), 3)}, ' \
                 f'{np.round(np.mean(fits), 3)}, ' \
                 f'{np.round(np.min(mountain_top_distances), 3)}, ' \
                 f'{np.round(np.mean(touching_mountain), 3)}, ' \
                 f'{np.round(np.min(creature_sizes), 3)}, ' \
                 f'{np.round(np.max(creature_sizes), 3)}, ' \
                 f'{np.round(np.mean(creature_sizes), 3)}, ' \
                 f'{np.round(np.mean(links))}, ' \
                 f'{np.round(np.max(links))}\n'
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
            if cr.get_fitness() == max_fit:
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
