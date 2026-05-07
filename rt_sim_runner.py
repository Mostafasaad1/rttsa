import socket
import time
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    print("Connecting to CoppeliaSim ZMQ Remote API...")
    try:
        client = RemoteAPIClient()
        sim = client.getObject('sim')
    except Exception as e:
        print(f"Error connecting to CoppeliaSim: {e}")
        return

    # Get joint handles dynamically from the /UR5 tree
    try:
        ur5_base = sim.getObject('/UR5')
        joint_handles = sim.getObjectsInTree(ur5_base, sim.sceneobject_joint, 0)
        if ur5_base in joint_handles:
            joint_handles.remove(ur5_base)
        if len(joint_handles) < 6:
            print(f"Error: Expected 6 joints, found {len(joint_handles)}.")
            return
        joint_handles = joint_handles[:6]
        print(f"Found {len(joint_handles)} joints: {joint_handles}")
    except Exception as e:
        print(f"Error finding /UR5 joints: {e}")
        return

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', 5000))
    sock.settimeout(2.0)

    # Free-running physics — no setStepping, PID controllers handle the motion
    sim.startSimulation()
    print("Simulation started (free-running mode).")
    print("Listening for UDP stream on port 5000... (Ctrl+C to stop)")

    packet_count = 0
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                positions_str = data.decode('utf-8').strip().split(',')
                if len(positions_str) == 6:
                    positions = [float(x) for x in positions_str]
                    for j in range(6):
                        sim.setJointTargetPosition(joint_handles[j], positions[j])
                    packet_count += 1
                    if packet_count % 20 == 1:
                        print(f"  [{packet_count}] joints: {[round(p, 3) for p in positions]}")

            except socket.timeout:
                if packet_count > 0:
                    print(f"Stream paused after {packet_count} packets.")
                    packet_count = 0
                else:
                    print("Waiting for bezier_ik_runner...")

            except KeyboardInterrupt:
                break

    except Exception as e:
        print(f"Error: {e}")

    print("Stopping simulation...")
    sim.stopSimulation()
    sock.close()

if __name__ == "__main__":
    main()
