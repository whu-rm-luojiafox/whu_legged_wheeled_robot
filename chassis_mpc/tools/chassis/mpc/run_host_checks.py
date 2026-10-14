"""Reproduce generation, native host build and numerical tests."""
from pathlib import Path
import argparse,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[3]
def main():
    p=argparse.ArgumentParser(); p.add_argument("--cxx",type=Path)
    p.add_argument("--regenerate",action="store_true",help="Regenerate solver caches explicitly")
    p.add_argument("--build-dir",type=Path,default=ROOT/"build/host")
    a=p.parse_args(); cxx=a.cxx
    if cxx is None:
        found=shutil.which("g++")
        if found: cxx=Path(found)
        else: cxx=next(Path("D:/clion-toolchain").glob("**/mingw64/bin/g++.exe"),None)
    if cxx is None or not cxx.is_file(): raise SystemExit("Specify a native C++ compiler with --cxx")
    def run(args): subprocess.run([str(x) for x in args],cwd=ROOT,check=True)
    if a.regenerate:
        run([sys.executable,ROOT/"tools/chassis/mpc/generate_solver.py"])
    run(["cmake","-S",ROOT,"-B",a.build_dir,"-G","Ninja","-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_CXX_COMPILER="+str(cxx),"-DCMAKE_C_COMPILER="+str(cxx.with_name("gcc.exe" if cxx.suffix == ".exe" else "gcc"))])
    run(["cmake","--build",a.build_dir,"-j","2"])
    run(["ctest","--test-dir",a.build_dir,"--output-on-failure"])
if __name__=="__main__": main()
