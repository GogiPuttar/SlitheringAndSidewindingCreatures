import numpy as np
from generate_random_bitstring import generate_random_bitstring
from evaluate_fitness import evaluate_fitness
from render_creature import render_creature
import pickle
import matplotlib.pyplot as plt

# Function to initialize population
def initialize_population(population_size):
    population = []

    # Hyperparameters
    body_length = np.random.randint(4, 20)
    spin_prob = 0.3

    for _ in range(population_size):
        gene = {
            'body_plan': generate_random_bitstring(body_length, spin_prob),
            'frequency': np.random.uniform(0.005, 0.05),
            'joint_range': np.random.uniform(10, 90),
            'motor_gain': np.random.uniform(15, 100)
        }
        population.append(gene)
    return population

# Function for selection (roulette wheel selection)
def selection(population, num_parents):

    fitness_values = [evaluate_fitness(creature) for creature in population]
    total_fitness = sum(fitness_values)
    probabilities = [fitness / total_fitness for fitness in fitness_values]
    selected_indices = np.random.choice(len(population), size=num_parents, replace=False, p=probabilities)
    
    return [population[index] for index in selected_indices]

# Function for crossover (single point crossover)
def single_point_crossover(parent1, parent2):

    crossover_point = np.random.randint(1, min(len(parent1['body_plan']), len(parent2['body_plan'])))
    child1 = {
        'body_plan': np.concatenate((parent1['body_plan'][:crossover_point], parent2['body_plan'][crossover_point:])),
        'frequency': (parent1['frequency'] + parent2['frequency']) / 2,
        'joint_range': (parent1['joint_range'] + parent2['joint_range']) / 2,
        'motor_gain': (parent1['motor_gain'] + parent2['motor_gain']) / 2
    }
    child2 = {
        'body_plan': np.concatenate((parent2['body_plan'][:crossover_point], parent1['body_plan'][crossover_point:])),
        'frequency': (parent1['frequency'] + parent2['frequency']) / 2,
        'joint_range': (parent1['joint_range'] + parent2['joint_range']) / 2,
        'motor_gain': (parent1['motor_gain'] + parent2['motor_gain']) / 2
    }
    return child1, child2

def crossover(parent1, parent2):
    # Calculate the average length of bitstrings
    avg_length = (len(parent1['body_plan']) + len(parent2['body_plan'])) // 2
    
    # Perform crossover
    child1_bitstring = []
    child2_bitstring = []
    for bit1, bit2 in zip(parent1['body_plan'], parent2['body_plan']):
        if bit1 == 0 and bit2 == 0:
            child1_bitstring.append(0)
            child2_bitstring.append(0)
        elif bit1 == 1 and bit2 == 1:
            child1_bitstring.append(1)
            child2_bitstring.append(1)
        else:
            child1_bitstring.append(0)
            child2_bitstring.append(0)
    
    # Trim or extend bitstrings to match the average length
    if len(child1_bitstring) > avg_length:
        child1_bitstring = child1_bitstring[:avg_length]
    elif len(child1_bitstring) < avg_length:
        child1_bitstring.extend([0] * (avg_length - len(child1_bitstring)))
    
    if len(child2_bitstring) > avg_length:
        child2_bitstring = child2_bitstring[:avg_length]
    elif len(child2_bitstring) < avg_length:
        child2_bitstring.extend([0] * (avg_length - len(child2_bitstring)))
    
    child1 = {
        'body_plan': np.array(child1_bitstring),
        'frequency': (parent1['frequency'] + parent2['frequency']) / 2,
        'joint_range': (parent1['joint_range'] + parent2['joint_range']) / 2,
        'motor_gain': (parent1['motor_gain'] + parent2['motor_gain']) / 2
    }
    child2 = {
        'body_plan': np.array(child2_bitstring),
        'frequency': (parent1['frequency'] + parent2['frequency']) / 2,
        'joint_range': (parent1['joint_range'] + parent2['joint_range']) / 2,
        'motor_gain': (parent1['motor_gain'] + parent2['motor_gain']) / 2
    }
    return child1, child2

# Function for mutation
def mutate(creature, mutation_rate):
    mutated_creature = creature.copy()
    for key in mutated_creature.keys():
        if np.random.rand() < mutation_rate:
            if key == 'body_plan':
                # print("Prior to mutating", mutated_creature[key])
                mutation_mask = np.random.rand(len(mutated_creature[key])) < mutation_rate
                # print("Mutation mask", mutation_mask)
                mutated_creature[key][mutation_mask] = 1 - mutated_creature[key][mutation_mask]
            else:
                mutated_creature[key] *= np.random.uniform(0.9, 1.1)
    return mutated_creature

# Genetic algorithm
def genetic_algorithm(population_size, num_generations):

    best_fitnesses = []
    average_fitnesses = []

    population = initialize_population(population_size)

    for generation in range(num_generations):

        print(f"Generation {generation + 1}/{num_generations}")

        fitness_values = [evaluate_fitness(creature) for creature in population]
        best_fitness = max(fitness_values)
        average_fitness = np.mean(fitness_values)
        best_fitnesses.append(best_fitness)
        average_fitnesses.append(average_fitness)

        parents = selection(population, population_size // 2)
        offspring = []

        for _ in range(population_size // 2):

            parent1, parent2 = np.random.choice(parents, size=2, replace=False)
            child1, child2 = crossover(parent1, parent2)
            offspring.extend([mutate(child1, mutation_rate=0.1), mutate(child2, mutation_rate=0.1)])
        
        population = offspring
    
    return population, best_fitnesses, average_fitnesses

def pick_best_creature(final_population):
    sorted_population = sorted(final_population, key=lambda x: evaluate_fitness(x), reverse=True)
    best_creature = sorted_population[0]  # The first creature has the highest fitness after sorting
    best_fitness = evaluate_fitness(best_creature)
    return best_creature, best_fitness

def save_best_creature(best_creature, filename):
    with open(filename, 'wb') as f:
        pickle.dump(best_creature, f)

# Example usage
population_size = 20
num_generations = 100

final_population, best_fitnesses, average_fitnesses = genetic_algorithm(population_size, num_generations)
# print("Final population:")
# for i, creature in enumerate(final_population):
#     print(f"Creature {i + 1}: {creature}")

# Example usage
best_creature, best_fitness = pick_best_creature(final_population)
print("Best creature:", best_creature)
print("Fitness:", best_fitness)

save_best_creature(best_creature, 'best_creature.pkl')

render_creature(best_creature)

# Plotting
generation_numbers = range(1, num_generations + 1)
plt.plot(generation_numbers, best_fitnesses, label='Best Fitness')
plt.plot(generation_numbers, average_fitnesses, label='Average Fitness')
plt.xlabel('Generation')
plt.ylabel('Fitness')
plt.title('Best and Average Fitness Across Generations')
plt.legend()
plt.grid(True)
plt.show()
