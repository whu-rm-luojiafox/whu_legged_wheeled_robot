"""Compare official-core runtime against independently condensed convex QP.
Host timing is NOT F407 timing. No firmware or motor output is modified.
"""
from pathlib import Path
import argparse
import json
import subprocess
import os
import numpy as np
from scipy.linalg import block_diag, cholesky, solve_triangular
from scipy.optimize import lsq_linear

ROOT=Path(__file__).resolve().parents[3]

def prediction(A,B,x0,intervals):
    free=[]; rows=[]
    for k in range(1,intervals+1):
        free.append(np.linalg.matrix_power(A,k)@x0)
        row=np.zeros((10,4*intervals))
        for i in range(k): row[:,4*i:4*i+4]=np.linalg.matrix_power(A,k-i-1)@B
        rows.append(row)
    return np.concatenate(free),np.vstack(rows)

def reference_qp(model,Qf,x0,ref,constrained):
    A,B,Q,R=[np.array(model[k]) for k in ["Ad","Bd","Q","R"]]
    free,S=prediction(A,B,x0,7)
    W=block_diag(*([Q]*6+[Qf])); Rbar=block_diag(*([R]*7))
    e=free-np.tile(ref,7); H=S.T@W@S+Rbar; g=S.T@W@e
    if constrained:
        C=cholesky(H,lower=True).T
        target=-solve_triangular(C.T,g,lower=True)
        limits=np.tile([6,6,12,12],7)
        opt=lsq_linear(C,target,bounds=(-limits,limits),method="bvls",tol=1e-12,max_iter=2000)
        assert opt.success,opt.message
        u=opt.x
    else: u=np.linalg.solve(H,-g)
    return u,H,g

def native(exe,cases,constrained,max_iter,tol):
    text="\n".join(" ".join(format(v,".17g") for v in np.r_[x,ref]) for _,x,ref in cases)+"\n"
    proc=subprocess.run([str(exe),"constrained" if constrained else "unconstrained",str(max_iter),str(tol)],
                        input=text,text=True,capture_output=True,check=True,timeout=120)
    rows=[np.array([float(v) for v in line.split()]) for line in proc.stdout.splitlines() if line.strip()]
    assert len(rows)==len(cases) and all(len(row)==36 for row in rows),proc.stdout
    return rows

def rollout(exe,model):
    A,B,K=[np.array(model[k]) for k in ["Ad","Bd","Kd"]]
    x=np.zeros(10); x[8]=.06
    lqr=x.copy(); limits=np.array([6,6,12,12]); rows=[]
    proc=subprocess.Popen([str(exe),"constrained","200","0.001"],stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    try:
        for step in range(500):
            if step==100:
                x[8]+=.04; x[2]+=.1; lqr[8]+=.04; lqr[2]+=.1
            proc.stdin.write(" ".join(format(v,".17g") for v in np.r_[x,np.zeros(10)])+"\n")
            proc.stdin.flush()
            values=np.array([float(v) for v in proc.stdout.readline().split()])
            assert len(values)==36 and values[1]==1,(step,values)
            u=values[8:12]; ulqr=np.clip(-K@lqr,-limits,limits)
            rows.append({"time_s":step*.004,"mpc_state":x.tolist(),"lqr_state":lqr.tolist(),
                         "mpc_input":u.tolist(),"lqr_input":ulqr.tolist(),"iterations":int(values[2])})
            x=A@x+B@u; lqr=A@lqr+B@ulqr
        proc.stdin.close(); proc.wait(timeout=10)
        assert proc.returncode==0
    finally:
        if proc.poll() is None: proc.kill(); proc.wait()
    return {"samples":rows,"final_mpc_state":x.tolist(),"final_lqr_state":lqr.tolist(),
            "description":"2-second exact linear-model replay; pitch/yaw impulse at 0.4 s; clipped full discrete LQR comparison; not hardware validation"}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--build-dir",type=Path,default=ROOT/"build/host")
    ap.add_argument("--output-dir",type=Path,default=ROOT/"build/host/reports")
    args=ap.parse_args()
    model=json.loads(next((ROOT/"tools/chassis/model/generated").glob("fixed_leg_L0p240_R0p240_Ts0p004000.json")).read_text(encoding="utf-8"))
    manifest=json.loads((ROOT/"src/module/chassis/chassis_mpc/mpc_generated/manifest.json").read_text())
    Qf=np.array(manifest["Qf"])
    def state(**values):
        x=np.zeros(10,dtype=np.float32)
        for name,value in values.items(): x[model["state_names"].index(name)]=value
        return x.astype(float)
    cases=[("zero",state(),state()),("pitch_small",state(pitch=.01),state()),
           ("yaw_small",state(yaw=.05),state()),("forward",state(ds=.2),state()),
           ("large_disturbance",state(pitch=.2,dpitch=.5,yaw=.6),state()),
           ("reference",state(),state(s=.1,yaw=.2))]
    records=[]
    for precision in ["double","float"]:
        exe=args.build_dir/("mpc_probe_"+precision+(".exe" if os.name == "nt" else ""))
        for constrained in [False,True]:
            rows=native(exe,cases,constrained,5000,1e-5 if precision=="double" else 1e-3)
            for (name,x,ref),row in zip(cases,rows):
                target,H,g=reference_qp(model,Qf,x,ref,constrained)
                u=row[8:]; error=float(np.max(abs(u-target)))
                gap=float((.5*u@H@u+g@u)-(.5*target@H@target+g@target))
                passed=bool(row[0]==0 and error < (0.003 if precision=="double" else .03))
                records.append({"precision":precision,"constraints":constrained,"case":name,
                    "status":int(row[0]),"valid":int(row[1]),"iterations":int(row[2]),
                    "host_time_us":row[3],"primal_residual":row[4],"dual_residual":row[5],
                    "input_bound_violation":row[6],"matrix_payload_bytes":int(row[7]),
                    "max_sequence_error_vs_qp":error,"objective_gap":gap,"pass":passed,
                    "first_control":u[:4].tolist(),"lqr_first_control":(-np.array(model["Kd"])@(x-ref)).tolist()})
    # Production-candidate budget: failure is expected/recorded, never hidden.
    budget=native(args.build_dir/("mpc_probe_float"+(".exe" if os.name == "nt" else "")),cases,True,200,.001)
    failure=native(args.build_dir/("mpc_probe_float"+(".exe" if os.name == "nt" else "")),cases[1:],True,1,1e-6)
    assert all(row[0]==3 and row[1]==0 for row in failure)
    report={"upstream_commit":manifest["upstream_commit"],"test":"fixed leg; 8 state knots; corrected official ADMM core; independent bounded least squares QP",
            "terminal_weight":"augmented DARE Pinf minus rho I (not exported Pd)",
            "records":records,"all_reference_checks_pass":all(r["pass"] for r in records),
            "budget_200_iterations":[{"case":case[0],"status":int(row[0]),"valid":int(row[1]),"iterations":int(row[2]),"host_time_us":row[3]} for case,row in zip(cases,budget)],
            "failure_guard_checks":len(failure),
            "linear_closed_loop":rollout(args.build_dir/("mpc_probe_float"+(".exe" if os.name == "nt" else "")),model),
            "limitations":["Host timing only; no board connected.","Dynamic Eigen allocation remains.","Model hip limits are experimental and do not ensure physical joint torque limits."]}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/"comparison.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print("Reference checks:",sum(r["pass"] for r in records),"/",len(records))
    for r in records:
        if not r["pass"]: print("FAIL",r["precision"],r["constraints"],r["case"],r["status"],r["max_sequence_error_vs_qp"])
    print("200-iteration valid:",sum(row[1] for row in budget),"/",len(budget))
    print("Report:",args.output_dir/"comparison.json")
    if not report["all_reference_checks_pass"]: raise SystemExit(1)

if __name__=="__main__": main()
