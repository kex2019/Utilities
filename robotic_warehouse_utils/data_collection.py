import sys
import pandas as pd
import time


class EvaluationDone(Exception):
    pass


def initGymCollect(gym, data, output, name, steps, collect):
    if collect:
        return GymCollect(gym, data, output, name, steps)
    else:
        return gym


class GymCollect(object):
    def __init__(
            self,
            gym: "RoboticWareHouse" = None,
            data: "???" = None,
            output: str = None,
            name: str = "Generic101",
            # Negative steps -> no bound
            steps: int = 10000):
        """ Hacky inheritance. """
        self.reset = lambda: gym.reset()
        self.render = lambda: gym.render()
        self.map = gym.map
        self.in_map = lambda y, x: gym.in_map(y, x)
        self.gym = gym

        self.PICKUP_INSTRUCTION = gym.PICKUP_INSTRUCTION
        self.DROP_INSTRUCTION = gym.DROP_INSTRUCTION
        self.UP_INSTRUCTION = gym.UP_INSTRUCTION
        self.DOWN_INSTRUCTION = gym.DOWN_INSTRUCTION
        self.LEFT_INSTRUCTION = gym.LEFT_INSTRUCTION
        self.RIGHT_INSTRUCTION = gym.RIGHT_INSTRUCTION
        """ Validate data source. """
        if type(data) == pd.DataFrame:
            self.df = data
        elif type(data) == str:
            self.df = pd.read_csv(data)
        else:
            print("Unsupported data type {}: {}".format(type(data), data))
            sys.exit(1)

        if list(self.df.columns):
            if len(self.df.columns) != 5:
                raise Exception("Dataframe has to many columns.")
            elif "points" in list(self.df.columns) \
                    and "timestamp" in list(self.df.columns)\
                    and "mean_decision_time" in list(self.df.columns)\
                    and "steps" in list(self.df.columns):
                pass
            else:
                raise Exception("Dataframe is missing column")
        self.output = output
        self.name = name

        self.last_action = time.time()
        self.action_spacing = []

        self.latency_meta = pd.DataFrame()
        self.collision_meta = pd.DataFrame()

        self.simulation_meta = pd.DataFrame({
            "robots": [self.gym.num_robots],
            "spawn": [self.gym.initial_spawn],
            "capacity": [self.gym.capacity],
            "seed": [self.gym.seed]
        })

        self.total_steps = 0
        self.step_cap = steps

    def save(self):
        if hasattr(self, "df"):
            self.df.to_csv(
                "{}/throughput_{}.csv".format(self.output, self.name),
                index=False)
            self.latency_meta.to_csv(
                "{}/latency_{}.csv".format(self.output, self.name),
                index=False)
            self.collision_meta.to_csv(
                "{}/collision_{}.csv".format(self.output, self.name),
                index=False)
            self.simulation_meta.to_csv(
                "{}/simulation_{}.csv".format(self.output, self.name),
                index=False)

    def step(self, *args, **kwargs):
        self.action_spacing.append(time.time() - self.last_action)
        self.last_action = time.time()
        self.total_steps += 1

        s, rew, e, i = self.gym.step(*args, **kwargs)
        if self.gym.round_collisions > 0:
            data_point = {
                "timestamp": time.time(),
                "steps": self.total_steps,
                "collisions": self.gym.round_collisions
            }

            self.collision_meta = self.collision_meta.append(
                data_point, ignore_index=True)

        if self.gym.round_dropoffs:
            for dropoff in self.gym.round_dropoffs:
                data_point = {
                    "timestamp": time.time(),
                    "steps": self.total_steps,
                    "latency": self.gym.steps - dropoff.spawn,
                    "dropped": 1
                }

                self.latency_meta = self.latency_meta.append(
                    data_point, ignore_index=True)

        if rew != 0:
            data_point = {
                "timestamp":
                time.time(),
                "points":
                rew,
                "mean_decision_time":
                sum(self.action_spacing) / len(self.action_spacing),
                "steps":
                self.total_steps
            }

            self.df = self.df.append(data_point, ignore_index=True)

        if self.total_steps == self.step_cap:
            print("{} steps reached, saving & stopping.".format(
                self.total_steps))
            print("See {} for saved data".format(self.output))

            for package in s[1]:
                data_point = {
                    "timestamp": time.time(),
                    "steps": self.total_steps,
                    "latency": self.gym.steps - package.spawn,
                    "dropped": 0
                }

                self.latency_meta = self.latency_meta.append(
                    data_point, ignore_index=True)

            self.save()

            raise EvaluationDone()
        return s, rew, e, i
