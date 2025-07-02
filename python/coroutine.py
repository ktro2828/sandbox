import asyncio
import numpy as np
import time


async def hello(x):
    def run(n):
        for i in range(3):
            print(f"hello {x}: {i}@{n}")
    return await asyncio.gather(*[run(n) for n in range(3)])


async def goodbye(x) -> None:
    def run(n):
        for i in range(3):
            print(f"goodbye {x}: {i}@{n}")
    return await asyncio.gather(*[run(n) for n in range(3)])


async def create_gather():
    async def _create_gather1() -> None:
        return await asyncio.gather(hello("foo"), goodbye("foo"))

    async def _create_gather2() -> None:
        return await asyncio.gather(hello("bar"), goodbye("bar"))

    g1 = _create_gather1()
    g2 = _create_gather2()
    gather = await asyncio.gather(g1, g2)

    print("end")
    return gather


def run_gather() -> None:
    gather = create_gather()
    print("start")
    asyncio.run(gather)


# def add(x) -> float:
#     return x + 2


# def check_yield(items):
#     for v in items:
#         yield add(v) % 2 == 0


# def check_booleans(items):
#     return [add(v) % 2 == 0 for v in items]


# def main():
#     items = np.random.rand(1000)
#     start1 = time.perf_counter_ns()
#     all(check_yield(items))
#     time1 = time.perf_counter_ns() - start1

#     start2 = time.perf_counter_ns()
#     all(check_booleans(items))
#     time2 = time.perf_counter_ns() - start2

#     start3 = time.perf_counter_ns()
#     all(add(v) % 2 == 0 for v in items)
#     time3 = time.perf_counter_ns() - start3

#     print(f"yield: {time1}, booleans: {time2}, comprehension: {time3}")


if __name__ == "__main__":
    # main()
    # print("Run Tasks:")
    # run_tasks()
    print("Run Gather:")
    run_gather()
