import robotic_warehouse.robotic_warehouse as warehouse
import robotic_warehouse_utils.path_finder as path_finder
import random
import time

timestamp = time.time()
gym = warehouse.RoboticWarehouse(
    robots=1,
    capacity=1,
    spawn=10,
    shelve_length=2,
    shelve_height=2,
    shelve_width=2,
    shelve_throughput=1,
    cross_throughput=5,
    seed=105,
    periodicity_lower=20,
    periodicity_upper=100)
print("Setup Time: {}".format(time.time() - timestamp))

pf = path_finder.Astar(gym)

steps = 0
timestamp = time.time()
try:
    instructions = []
    I = 0
    finding = False
    while True:
        gym.render()
        if len(instructions) <= I and finding:
            (robots, packages), _, _, _ = gym.step([gym.PICKUP_INSTRUCTION])

            current_position = robots[0][0]
            target_position = pf.available_pos_near(robots[0][1][0])

            instructions = pf(current_position,
                              target_position).get_instructions()
            I = 0
            finding = False
        elif len(instructions) <= I:
            (robots, packages), _, _, _ = gym.step([gym.DROP_INSTRUCTION])
            current_position = robots[0][0]
            if packages:
                target_position = pf.available_pos_near(
                    random.choice(packages)[0])
                instructions = pf(current_position,
                                  target_position).get_instructions()
                I = 0

                finding = True
        else:
            gym.step([instructions[I]])
            I += 1

        steps += 1
except KeyboardInterrupt:
    print("Number of steps: {}, average step per second: {}".format(
        steps, steps / (time.time() - timestamp)))
