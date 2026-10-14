python3 - <<'INNER'
import subprocess,os,json
from pathlib import Path
code="""import rclpy,time,json
rclpy.init();n=rclpy.create_node('post_replay_cleanup_probe');end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
others=[(a,b) for a,b in n.get_node_names_and_namespaces() if a!=n.get_name()]
n.destroy_node();rclpy.shutdown();print(json.dumps(others));assert not others,others
"""
r={}
for domain in [191,192,193,194]:
 p=subprocess.run(['python3','-c',code],env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),capture_output=True,text=True,timeout=10)
 r[domain]={'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr};assert p.returncode==0,p.stderr
Path('/review-evidence/replay-domains-empty.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r))
INNER
