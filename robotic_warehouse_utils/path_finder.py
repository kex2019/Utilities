import heapq
import math


def l1norm_dist(p1: [], p2: []) -> float:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def l2norm_dist(p1: [], p2: []) -> float:
    return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                     (p1[1] - p2[1]) * (p1[1] - p2[1]))


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
        self.mem_size = ((self.height + 1) * self.width)
        self.mem_range = range(self.mem_size)
        self.mem = [-1] * self.mem_size

        self.D = [gym.UP, gym.DOWN, gym.LEFT, gym.RIGHT]

        self.p1 = None
        self.p2 = None

    def __2d_mem(self, p: []) -> int:
        return p[0] * self.width + p[1]

    def __mem_2d(self, p: int) -> []:
        y = p // self.width
        x = p % self.width
        return [y, x]

    def __is_ok_to_walk(self, y: int, x: int):
        return self.gym.map[y][x][0] == self.gym.TILE_ID

    def available_pos_near(self, p: []) -> []:
        if self.__is_ok_to_walk(*p):
            return p

        for d in self.D:
            if self.__is_ok_to_walk(d[0] + p[0], d[1] + p[1]):
                return [d[0] + p[0], d[1] + p[1]]

    def __fill_mem(self, p1: [], p2: []) -> None:
        """ populate mem to enable instruction and position retrieval"""
        self.p1 = p1
        self.p2 = p2
        """ First reset memory. """
        for i in self.mem_range:
            self.mem[i] = -1
        """ Then do Astar. """
        pq = []

        self.mem[self.__2d_mem(self.p1)] = self.__2d_mem(self.p1)
        heapq.heapify(pq)
        for d in self.D:
            p = [d[0] + p1[0], d[1] + p1[1]]
            if self.gym.in_map(*p) and self.__is_ok_to_walk(*p):
                self.mem[self.__2d_mem(p)] = self.__2d_mem(p1)
                heapq.heappush(
                    pq,
                    # Search Score                   #Steps #Point
                    (self.norm_weight * self.norm(p, p2), 1, p))

        while pq:
            score, steps, pp = heapq.heappop(pq)

            if pp == p2:
                break

            for d in self.D:
                p = [d[0] + pp[0], d[1] + pp[1]]
                if self.gym.in_map(*p) and self.__is_ok_to_walk(
                        *p) and self.mem[self.__2d_mem(p)] == -1:
                    self.mem[self.__2d_mem(p)] = self.__2d_mem(pp)
                    heapq.heappush(
                        pq, (steps + self.norm_weight * self.norm(p, p2),
                             steps + 1, p))

        return None

    def __call__(self, p1: [], p2: []) -> "Astar":
        self.__fill_mem(p1, p2)
        return self

    def get_instructions(self) -> [int]:
        inst = []
        p = self.p2
        while list(p) != list(self.p1):
            pn = self.__mem_2d(self.mem[int(self.__2d_mem(p))])
            step = [p[0] - pn[0], p[1] - pn[1]]

            m1 = self.__2d_mem(p)
            m2 = self.mem[int(m1)]
            m3 = self.__mem_2d(m2)

            if step == self.gym.UP:
                inst.append(self.gym.UP_INSTRUCTION)
            elif step == self.gym.DOWN:
                inst.append(self.gym.DOWN_INSTRUCTION)
            elif step == self.gym.LEFT:
                inst.append(self.gym.LEFT_INSTRUCTION)
            elif step == self.gym.RIGHT:
                inst.append(self.gym.RIGHT_INSTRUCTION)
            else:
                print(
                    "ERROR.. add logging? - get instruction in pathfinder - got {}".
                    format(step))
                break

            p = pn

        inst.reverse()
        return inst

    def get_positions(self) -> [[]]:
        pos = []
        p = p2
        while any(p != self.p1):
            pn = self.__mem_2d(self.mem[self.__2d_mem(p)])
            pos.append(p)
            p = pn

        pos.reverse()
        return pos
