import time
import timeit

start = time.time()
x = 0
while not x == 100:
    ptime = time.time() - start
    print(ptime)
    x += 1
start = timeit.default_timer()
x = 0
while not x == 100:
    ptime = timeit.default_timer() - start
    print(ptime)
    x += 1