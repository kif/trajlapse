import glob, time, os
from trajlapse.analysis  import Analyzer
from multiprocessing import Process, Queue

# glob.glob("/home/kieffer/Pictures/2022-06-14-21h51m14s/*.jpg")
# files = glob.glob("/home/kieffer/Pictures/2022-06-14-21h51m14s/*.jpg")
# print(len(files))
# a = Analyzer((3040, 4056))
# a.process(files[0])


def analyzer(shape, qin, qout):
    "Simple analyzer process"
    from trajlapse.analysis  import Analyzer
    fname = qin.get() 
    a = Analyzer(shape)
    while fname is not None:
        if fname and os.path.exists(fname):
            res = a.process(fname)
            qout.put(res)
        fname = qin.get()


def process_many(filenames, nproc=4):
    qin = Queue()
    qout = Queue()
    pool = [Process(target=analyzer, args=((3040, 4056), qin, qout))
            for i in range(nproc)]
    for p in pool:
        p.start()
    for fn in filenames:
        # print(fn)
        qin.put(fn)
    # kill remaining processes:
    for i in range(nproc):
        qin.put(None)
    for p in pool:
        p.join()
    return [qout.get() for i in filenames]


if __name__ == "__main__":
    nproc = 2
    files = glob.glob("/home/kieffer/Pictures/2022-06-14/*.jpg")
    print(len(files))
    t0 = time.perf_counter()
    results = process_many(files, nproc)
    t1 = time.perf_counter()
    print(f"performances: {(t1-t0)/len(files)} s/img using {nproc} processes")
    print(results[-1])
