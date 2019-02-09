import math
import heapq
import numpy as np


def l1norm_dist(p1: np.ndarray, p2: np.ndarray) -> float:
    return np.sum(np.abs(p1 - p2))


def l2norm_dist(p1: np.ndarray, p2: np.ndarray) -> float:
    return np.sqrt(np.sum(np.square(p1 - p2)))


def no_norm(_, __) -> float:
    return 0


class Astar(object):
    def __init__(self,
                 gym: "RoboticWareHouse",
                 norm: "F: R^2 x R^2 -> R" = l1norm_dist,
                 norm_weight: float = 1):
        self.gym = gym
        self.width = gym.map_width
        self.height = gym.map_height
        self.norm = norm
        self.norm_weight = norm_weight

        # If we want super big maps then we have to think of another way
        self.mem = np.ones((self.height + 1) * self.width) * -1

        self.D = [gym.UP, gym.DOWN, gym.LEFT, gym.RIGHT]

        self.p1 = None
        self.p2 = None

    def __2d_mem(self, p: np.ndarray) -> int:
        return p[0] * self.width + p[1]

    def __mem_2d(self, p: int) -> np.ndarray:
        y = p // self.width
        x = p % self.width
        return np.array([y, x])

    def __is_ok_to_walk(self, y: int, x: int):
        return self.gym.map[y][x][0] == self.gym.FREE_ID\
                or self.gym.map[y][x][0] == self.gym.ROBOT_ID

    def available_pos_near(self, p: np.ndarray) -> np.ndarray:
        if self.__is_ok_to_walk(*p):
            return p

        for d in self.D:
            pp = p + d
            if self.__is_ok_to_walk(*pp):
                return pp

    def __fill_mem(self, p1: np.ndarray, p2: np.ndarray) -> None:
        """ populate mem to enable instruction and position retrieval"""
        self.p1 = p1
        self.p2 = p2
        """ First reset memory. """
        self.mem.fill(-1)
        """ Then do Astar. """
        pq = []
        """ 
        Why this cmp_count thing one might ask.. that is a perfectly legitimate question..
        So.. there is no way to pass a comparator to the heapq and it will compare 
        tuples order wise... so if for some reason everything is equal except the saved
        point then the heapq will crash.. because comparing two numpy arrays returns
        a boolean array, a special case that the underlying heapq does not handle..
        and thus crashes... adding this counter ensures that the np arrays will never
        be compared
        """
        cmp_count = 0

        self.mem[self.__2d_mem(self.p1)] = 0
        heapq.heapify(pq)
        for d in self.D:
            p = d + p1
            if self.gym.in_map(*p) and self.__is_ok_to_walk(*p):
                self.mem[self.__2d_mem(p)] = self.__2d_mem(p1)
                heapq.heappush(
                    pq,
                    # Search Score                   #Steps #Point
                    (self.norm_weight * self.norm(p, p2), 1, cmp_count, p))
                cmp_count += 1

        while pq:
            score, steps, _, pp = heapq.heappop(pq)

            if all(pp == p2):
                break

            for d in self.D:
                p = d + pp
                if self.gym.in_map(*p) and self.__is_ok_to_walk(
                        *p) and self.mem[self.__2d_mem(p)] == -1:
                    self.mem[self.__2d_mem(p)] = self.__2d_mem(pp)
                    heapq.heappush(
                        pq, (steps + self.norm_weight * self.norm(p, p2),
                             steps + 1, cmp_count, p))
                    cmp_count += 1

        return None

    def __call__(self, p1: np.ndarray, p2: np.ndarray) -> "Astar":
        self.__fill_mem(p1, p2)
        return self

    def get_instructions(self) -> [int]:
        inst = []
        p = self.p2
        while any(p != self.p1):
            pn = self.__mem_2d(self.mem[int(self.__2d_mem(p))])
            step = p - pn

            m1 = self.__2d_mem(p)
            m2 = self.mem[int(m1)]
            m3 = self.__mem_2d(m2)

            if all(step == self.gym.UP):
                inst.append(self.gym.UP_INSTRUCTION)
            elif all(step == self.gym.DOWN):
                inst.append(self.gym.DOWN_INSTRUCTION)
            elif all(step == self.gym.LEFT):
                inst.append(self.gym.LEFT_INSTRUCTION)
            elif all(step == self.gym.RIGHT):
                inst.append(self.gym.RIGHT_INSTRUCTION)
            else:
                print("ERROR.. add logging? - get instruction")

            p = pn

        inst.reverse()
        return inst

    def get_positions(self) -> [np.ndarray]:
        pos = []
        p = p2
        while any(p != self.p1):
            pn = self.__mem_2d(self.mem[self.__2d_mem(p)])
            step = p - pn
            pos.append(p)
            p = pn

        pos.reverse()
        return pos
