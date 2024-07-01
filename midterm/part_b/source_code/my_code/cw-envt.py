import creature
import population
import simulation
import genome
import numpy as np

# Adjust basic variables for basic coursework
pop_size = 10
gene_count = 3
num_iterations = 101
sim_time = 2400

pop = population.Population(pop_size=pop_size, gene_count=gene_count)
sim = simulation.Simulation(sim_time=sim_time)

# Create csv_stats file to record stats of genetic algorithm.
csv_stats_file = open('stats.csv', 'x')
csv_stats_file.write('fittest, mean, mean links, max links\n')
csv_stats_file.close()

for iteration in range(num_iterations):
    for cr in pop.creatures:
        sim.run_creature(cr)

    fits = [cr.get_fitness() for cr in pop.creatures]

    links = [len(cr.get_expanded_links()) for cr in pop.creatures]

    # Write current population stats to stats.csv
    csv_stats_file = open('stats.csv', 'a')
    stats_line = f'{np.round(np.min(fits), 3)}, ' \
                 f'{np.round(np.mean(fits), 3)}, ' \
                 f'{np.round(np.mean(links))}, ' \
                 f'{np.round(np.max(links))}\n'
    csv_stats_file.write(stats_line)
    csv_stats_file.close()

    fit_map = population.Population.get_fitness_map(fits)
    new_creatures = []

    for i in range(len(pop.creatures)):
        p1_ind = population.Population.select_parent(fit_map)
        p2_ind = population.Population.select_parent(fit_map)
        p1 = pop.creatures[p1_ind]
        p2 = pop.creatures[p2_ind]

        # now we have the parents!
        dna = genome.Genome.crossover(p1.dna, p2.dna)
        dna = genome.Genome.point_mutate(dna, rate=0.1)
        dna = genome.Genome.shrink_mutate(dna, rate=0.25)
        dna = genome.Genome.grow_mutate(dna, rate=0.1)
        cr = creature.Creature(1)
        cr.update_dna(dna)
        new_creatures.append(cr)

    # elitism
    min_fit = np.min(fits)

    # Limit production of elite creatures with 10,000 iterations.
    if iteration % 10 == 0:
        # Get elite creature for given population.
        for cr in pop.creatures:
            if cr.get_fitness() == min_fit:
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
