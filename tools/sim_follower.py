# simulate leader (straight, arc left, straight, arc right) and a follower using decide()
import sys, math, types
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
for l in ("pigpio", "gpiozero"): sys.modules[l] = types.ModuleType(l)
sys.argv = ["follower.py", "40", "noir"]
import config, follower as F
T = config.TRACK_CM; dt = 0.05; k = F.CM_S_PER_RPM
def step(st, l, r):
    x, y, h, s = st; vl, vr = l*k, r*k; v = (vl+vr)/2; w = (vr-vl)/T
    return (x+v*math.cos(h)*dt, y+v*math.sin(h)*dt, h+w*dt, s+v*dt)
lead = (0.0, 0, 0, 0); fol = (-40.0, 0, 0, 0); path = F.Path(); where = F.Position(); recent = []
plan = [(10, (30, 30)), (8, (15, 30)), (6, (30, 30)), (8, (30, 15)), (6, (30, 30)), (4, (0, 0))]
t = 0; worst = 0; maxgap = 0; mingap = 1e9; log = [(x/2 - 40, 0) for x in range(81)]
for dur, (l, r) in plan:
    for _ in range(int(dur/dt)):
        lead = step(lead, l, r); t += dt
        if round(t/dt) % 2 == 0:  # packet every 0.1 s
            path.add(lead[3] - 0, lead[2]); recent = [p for p in recent if t-p[0] < 0.5] + [(t, lead[3])]
        frpm = ((recent[-1][1]-recent[0][1])/(recent[-1][0]-recent[0][0])/k) if len(recent) > 1 else 0
        where.update(fol[3], fol[2]); L, R, gap, err, st = F.decide(path, frpm, fol[3], fol[2], (where.x, where.y), None, True)
        fol = step(fol, L, R)
        # follower distance from the leader's actual path
        d = min(math.hypot(fol[0]-px, fol[1]-py) for px, py in [(lp[0], lp[1]) for lp in log] or [(fol[0], 0)]) if log else 0
        log.append(lead)
        straight = math.hypot(lead[0]-fol[0], lead[1]-fol[1])
        if t > 2: worst = max(worst, d); maxgap = max(maxgap, straight); mingap = min(mingap, straight)
print(f"follower off leader path max {worst:.2f} cm; axle gap straight line {mingap:.1f} to {maxgap:.1f} cm; final gap {math.hypot(lead[0]-fol[0], lead[1]-fol[1]):.1f}")
