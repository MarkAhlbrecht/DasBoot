# MP Test
import multiprocessing
import time

def proc1(sleepTime,name):
    print(f"Starting {name}:{sleepTime} sec ...")
    time.sleep(sleepTime)
    print(f"Done     {name}:{sleepTime}")



if (__name__ == "__main__"):

    start = time.perf_counter()
    procs = []
    nProcs = 1000
    
    for i in range(nProcs):
        pName = f"P{i}"
        p=multiprocessing.Process(target=proc1,args=[i/10,pName])
        procs.append(p)
        p.start()
    
    for p in procs:
        p.join()
    
    end = time.perf_counter()
    print(f"Elapsed Time {end-start}")