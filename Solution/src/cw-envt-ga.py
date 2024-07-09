import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import math

# Connect to PyBullet physics server in GUI mode
client = p.connect(p.GUI)
if client < 0:
    raise Exception("Failed to connect to the PyBullet physics server.")

p.setAdditionalSearchPath(pybullet_data.getDataPath())

def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size/4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    # Create four walls
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])

def evaluate_creature(creature_id):
    """Evaluate a creature and return its fitness score based on height achieved."""
    max_height = 0
    start_time = time.time()
    while time.time() - start_time < 10:  # Simulate for 10 seconds
        p.stepSimulation()
        pos, _ = p.getBasePositionAndOrientation(creature_id)
        max_height = max(max_height, pos[2])
        time.sleep(1./240.)  # Adjust to your simulation speed
    return max_height

def run_ga(gene_count=3, population_size=10, generations=10):
    """Run the genetic algorithm."""
    population = [creature.Creature(gene_count) for _ in range(population_size)]
    best_creature = None
    best_fitness = 0

    for generation in range(generations):
        fitness_scores = []
        for cr in population:
            # Save the creature to XML and load it into the simulation
            with open('test.urdf', 'w') as f:
                f.write(cr.to_xml())
            creature_id = p.loadURDF('test.urdf', (0, 0, 10))
            
            # Evaluate the creature
            fitness = evaluate_creature(creature_id)
            fitness_scores.append((cr, fitness))
            
            # Remove the creature from the simulation
            p.removeBody(creature_id)
        
        # Select the best creature
        fitness_scores.sort(key=lambda x: x[1], reverse=True)
        best_creature = fitness_scores[0][0]
        best_fitness = fitness_scores[0][1]

        # Print the best fitness of the generation
        print(f"Generation {generation}, Best Fitness: {best_fitness}")

        # Apply genetic algorithm operators (selection, crossover, mutation) to create the next generation
        next_population = []
        for _ in range(population_size):
            parent1, parent2 = random.choices(fitness_scores, k=2)
            child = parent1[0].crossover(parent2[0])
            child.mutate()
            next_population.append(child)
        population = next_population

    return best_creature, best_fitness

p.setGravity(0, 0, -10)

arena_size = 20
make_arena(arena_size=arena_size)

mountain_position = (0, 0, -1)  # Adjust as needed
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
p.setAdditionalSearchPath('shapes/')
mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

# Set the camera to a suitable position and orientation
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

# Run the genetic algorithm
best_creature, best_fitness = run_ga()

# Print the best creature's fitness
print(f"Best Creature Fitness: {best_fitness}")

# Visualize the best creature
with open('best_creature.urdf', 'w') as f:
    f.write(best_creature.to_xml())
best_creature_id = p.loadURDF('best_creature.urdf', (0, 0, 10))

p.setRealTimeSimulation(1)
while True:
    p.stepSimulation()
    time.sleep(1./240.)
