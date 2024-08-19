# Sweep the threshold voltage of EDLA remote logic analyser
#
# v0.01 JPB 12/5/22  First version
# v0.02 JPB 13/5/22  Use driver functions in edla_utils
# v0.03 JPB 14/5/22  Implemented voltage sweep

import pedla_utils as edla, base64, numpy as np

edla.verbose_mode(False)
#unit = edla.EdlaUnit(1, "192.168.8")
unit = edla.EdlaUnit(191, "10.1.1")
unit.set_sample_rate(10000)
unit.set_sample_count(10000)

MIN_V, MAX_V, STEP_V = 0, 5001, 500

def get_data():
    ok = False
    data = None
    status = unit.fetch_status()
    if status:
        ok = unit.do_capture()
    else:
        print("Can't fetch status from %s" % unit.status_url)
    if ok:
        data = unit.do_load()
    if data == None:
        print("Can't load data")
    return data

for v in range(MIN_V, MAX_V, STEP_V):
    unit.set_threshold(v)
    d = get_data()
    byts = base64.b64decode(d)
    samps = np.frombuffer(byts, dtype=np.uint16)
    # print("Loaded %u samples" % len(samps))
    diffs = np.diff(samps)
    edges = np.where(diffs != 0)[0]
    # print("%u edges" % len(edges))
    totals = np.zeros(16, dtype=int)
    for edge in edges:
        bits = samps[edge] ^ samps[edge+1]
        for n in range(0, 15):
            if bits & (1<<n):
                totals[n] += 1
    s = "%4u," % v
    s += ",".join([("%4u" % val) for val in totals])
    print(s)

# EOF
