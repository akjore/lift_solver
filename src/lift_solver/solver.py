import numpy as np
from scipy.optimize import minimize, NonlinearConstraint
#import pyvista as pv

# placeholder
g = 9.81
m_load = 2000
m_sp = 300

W_load = m_load * g
W_sp = m_sp * g

k = np.ones(6) * 1e5
L0 = np.ones(6) * 4.0

apex = np.array([0.0, 0.0, 10.0])
#cog_offset = np.array([0.5, 0.5, 0.0])
cog_offset = np.array([0.0, 0.0, 0.0])

sp_local = np.array([
        [0, -1.5,  0.25],
        [0,  1.5,  0.25],
        [0, -1.5, -0.25],
        [0,  1.5, -0.25],
    ])

ld_local = np.array([
        [-2.5, -1.5, 1],
        [ 2.5, -1.5, 1],
        [-2.5,  1.5, 1],
        [ 2.5,  1.5, 1],
    ])


# -------------------------
def transform(points, pos, ang):
    r,p,y = ang
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    R=Rz@Ry@Rx
    return np.array([pos+R@pt for pt in points])

# -------------------------
def sling_data(x):
    sp_pos,sp_ang = x[0:3],x[3:6]
    ld_pos,ld_ang = x[6:9],x[9:12]
    T = x[12:18]

    sp_pts = transform(sp_local, sp_pos, sp_ang)
    ld_pts = transform(ld_local, ld_pos, ld_ang)

    TL,TR,BL,BR = sp_pts
    pairs = [
        (apex, TL), (apex, TR),
        (BL, ld_pts[0]), (BL, ld_pts[1]),
        (BR, ld_pts[2]), (BR, ld_pts[3])
    ]

    F=[]; Ls=[]
    for i,(a,b) in enumerate(pairs):
        d=b-a
        L=np.linalg.norm(d)
        if L < 1e-8:
            Fi=np.zeros(3)
        else:
            Fi = -T[i] * d / L
        F.append(Fi); Ls.append(L)

    return np.array(F), np.array(Ls), sp_pts, ld_pts, pairs

# -------------------------
def objective(x):
    T = x[12:18]
    return 0.5 * np.sum(T*T/k)

# -------------------------
def equilibrium(x):
    F,Ls,sp_pts,ld_pts,_ = sling_data(x)

    sp_pos=x[0:3]
    ld_pos=x[6:9]

    F_load=np.sum(F[2:],axis=0)+np.array([0,0,-W_load])
    cog=ld_pos+cog_offset
    M_load=sum(np.cross(p-cog,f) for p,f in zip(ld_pts,F[2:]))

    TL,TR,BL,BR=sp_pts
    F_sp=F[0]+F[1]-np.sum(F[2:],axis=0)+np.array([0,0,-W_sp])

    M_sp=(np.cross(TL-sp_pos,F[0])+np.cross(TR-sp_pos,F[1])+
          np.cross(BL-sp_pos,-(F[2]+F[3]))+
          np.cross(BR-sp_pos,-(F[4]+F[5])))

    return np.concatenate([F_load,M_load,F_sp,M_sp])

# -------------------------
def compatibility(x):
    T = x[12:18]
    _,Ls,_,_,_ = sling_data(x)
    return Ls - L0 - T/k


def solve(problem):
    # constraints
    neq_con = NonlinearConstraint(equilibrium, 0, 0)
    comp_con = NonlinearConstraint(compatibility, 0, 0)

    bounds = [(None,None)]*12 + [(0,None)]*6

    # initial guess
    x0 = np.zeros(18)
    x0[2] = 7
    x0[8] = 4
    x0[12:18] = 20000

    sol = minimize(objective, x0, method='trust-constr',
               bounds=bounds,
               constraints=[neq_con, comp_con],
               options={'verbose': 3})

    print("\n=== STATUS ===")
    print(sol.message)

    x = sol.x
    T = x[12:18]
    sp_pos,sp_ang = x[0:3],x[3:6]
    ld_pos,ld_ang = x[6:9],x[9:12]

    print("\n=== BODY STATES ===")
    print("Hook:",apex)
    print("Spreader:",sp_pos,np.degrees(sp_ang))
    print("Load:",ld_pos,np.degrees(ld_ang))

    F,Ls,sp_pts,ld_pts,pairs = sling_data(x)

    print("\n=== SLING FORCES ===")
    for i,Fi in enumerate(F):
        print(f"Sling {i}: F={Fi/1000} kN, T={T[i]/1000:.2f} kN")



    # ====================================================
    # FULL FORCE & MOMENT REPORTING
    # ====================================================

    # Load
    F_load=np.sum(F[2:],axis=0)+np.array([0,0,-W_load])
    cog_load=ld_pos+cog_offset
    M_load=sum(np.cross(p-cog_load,f) for p,f in zip(ld_pts,F[2:]))

    # Spreader
    F_sp=F[0]+F[1]-np.sum(F[2:],axis=0)+np.array([0,0,-W_sp])
    TL,TR,BL,BR=sp_pts
    M_sp=(np.cross(TL-sp_pos,F[0])+
          np.cross(TR-sp_pos,F[1])+
          np.cross(BL-sp_pos,-(F[2]+F[3]))+
          np.cross(BR-sp_pos,-(F[4]+F[5])))

    # Hook (reaction from slings)
    F_hook = -(F[0]+F[1])
    M_hook = np.zeros(3)

    print("\n=== FORCE BALANCE ===")
    print("Load ΣF:",F_load)
    print("Spreader ΣF:",F_sp)
    print("Hook force:",F_hook)

    print("\n=== MOMENT BALANCE ===")
    print("Load ΣM:",M_load)
    print("Spreader ΣM:",M_sp)
    print("Hook ΣM:",M_hook)


    return {
        "status": "ok",
        "n_lift_points": len(problem.get("lift_points", []))
    }

