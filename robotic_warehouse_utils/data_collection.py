import sys
import pandas as pd
import time


class EvaluationDone(Exception):
    pass


class GymCollect(object):
    def __init__(
            self,
            gym: "RoboticWareHouse" = None,
            data: "???" = None,
            output: str = None,
            name: str = "Generic101",
            # Negative steps -> no bound
            steps: int = 10000,
            collect=True):
        if collect:
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
                        and "name" in list(self.df.columns)\
                        and "mean_decision_time" in list(self.df.columns)\
                        and "steps" in list(self.df.columns)\
                        and "total_steps" in list(self.df.columns):
                    pass
                else:
                    raise Exception("Dataframe is missing column")
            """ ... """
            self.output = output
            self.name = name

            self.last_action = time.time()
            self.action_spacing = []

            self.total_steps = 0
            self.step_cap = steps

    def save(self):
        if hasattr(self, "df"):
            self.df.to_csv(self.output, index=False)

    def step(self, *args, **kwargs):
        self.action_spacing.append(time.time() - self.last_action)
        self.last_action = time.time()
        self.total_steps += 1

        s, rew, e, i = self.gym.step(*args, **kwargs)

        if rew != 0:
            data_point = {
                "timestamp":
                time.time(),
                "name":
                self.name,
                "points":
                rew,
                "mean_decision_time":
                sum(self.action_spacing) / len(self.action_spacing),
                "steps":
                len(self.action_spacing),
                "total_steps":
                self.total_steps
            }

            self.df = self.df.append(data_point, ignore_index=True)

        if self.total_steps == self.step_cap:
            print("{} steps reached, saving & stopping.".format(
                self.total_steps))
            print("See {} for saved data".format(self.output))
            self.save()

            if self.df["total_steps"].iloc[-1] < self.total_steps / 2:
                print(
                    "It is very likely that the algorithm made the robots get stuck"
                )
                print(
                    "{} steps was taken in total but no packages was delivered after {}".
                    format(self.total_steps, self.df["total_steps"].iloc[-1]))

            raise EvaluationDone()
        return s, rew, e, i
