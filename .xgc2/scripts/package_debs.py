#!/usr/bin/env python3
"""Package the installed standalone server and its owning ROS service only."""
import argparse, pathlib, shutil, subprocess

def package(stage, output, dependencies):
    stage=pathlib.Path(stage);output=pathlib.Path(output)
    server=stage/'opt/xgc2/xsim/bin/xsim'
    if not server.is_file():raise ValueError('installed xsim executable is required')
    ros=stage/'opt/ros/noetic'
    keep=['share/xgc2_lightweight_sim_msgs','include/xgc2_lightweight_sim_msgs',
          'lib/python3/dist-packages/xgc2_lightweight_sim_msgs','lib/pkgconfig/xgc2_lightweight_sim_msgs.pc']
    filtered=stage/'ros-interface-only'
    filtered.mkdir()
    for name in keep:
        source=ros/name
        if source.exists():
            target=filtered/name;target.parent.mkdir(parents=True,exist_ok=True)
            if source.is_dir():shutil.copytree(source,target)
            else:shutil.copy2(source,target)
    if not (filtered/'share/xgc2_lightweight_sim_msgs/package.xml').is_file():raise ValueError('owning SetProvider package is required')
    shutil.rmtree(ros);filtered.rename(ros)
    # catkin's global setup/.catkin are owned by Noetic, never by xsim.
    arch=subprocess.check_output(['dpkg','--print-architecture'],text=True).strip()
    control=stage/'DEBIAN';control.mkdir(exist_ok=True)
    (control/'control').write_text(f'''Package: xsim
Version: 1.0.0
Architecture: {arch}
Maintainer: XGC Team <dev@xgc.team>
Conflicts: xgc2-lightweight-sim, ros-noetic-xgc2-lightweight-sim-msgs
Replaces: xgc2-lightweight-sim, ros-noetic-xgc2-lightweight-sim-msgs
Depends: {dependencies}
Description: Standalone ECS world and integrated CPU/GPU point cloud server
''')
    subprocess.run(['dpkg-deb','--build','--root-owner-group',str(stage),str(output)],check=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',required=True);p.add_argument('--output',required=True);p.add_argument('--depends',required=True);a=p.parse_args()
    package(a.stage,a.output,a.depends)
if __name__=='__main__':main()
