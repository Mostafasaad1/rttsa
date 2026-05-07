"""
CoppeliaSim Bridge Module
Can be used standalone or integrated into the GUI
"""
import socket
import threading
import time
from typing import Optional, Callable

class SimBridge:
    def __init__(self, port=5000, log_callback: Optional[Callable[[str], None]] = None):
        self.port = port
        self.log_callback = log_callback or print
        self.running = False
        self.thread = None
        self.sock = None
        self.sim = None
        self.joint_handles = None
        self.packet_count = 0
        
    def log(self, message: str):
        """Log message using callback or print"""
        self.log_callback(message)
        
    def connect_coppeliasim(self):
        """Connect to CoppeliaSim via ZMQ Remote API"""
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient
            self.log("Connecting to CoppeliaSim ZMQ Remote API...")
            client = RemoteAPIClient()
            self.sim = client.getObject('sim')
            
            # Get joint handles dynamically from the /UR5 tree
            ur5_base = self.sim.getObject('/UR5')
            self.joint_handles = self.sim.getObjectsInTree(ur5_base, self.sim.sceneobject_joint, 0)
            if ur5_base in self.joint_handles:
                self.joint_handles.remove(ur5_base)
            if len(self.joint_handles) < 6:
                raise Exception(f"Expected 6 joints, found {len(self.joint_handles)}")
            self.joint_handles = self.joint_handles[:6]
            self.log(f"✓ Found {len(self.joint_handles)} joints")
            return True
        except Exception as e:
            self.log(f"✗ Error connecting to CoppeliaSim: {e}")
            return False
            
    def start(self):
        """Start the bridge in a background thread"""
        if self.running:
            self.log("Bridge already running")
            return False
            
        if not self.connect_coppeliasim():
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.log(f"✓ Bridge started on port {self.port}")
        return True
        
    def stop(self):
        """Stop the bridge"""
        if not self.running:
            return
            
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.sock:
            self.sock.close()
        if self.sim:
            try:
                self.sim.stopSimulation()
            except:
                pass
        self.log("✓ Bridge stopped")
        
    def _run_loop(self):
        """Main bridge loop (runs in background thread)"""
        try:
            # Setup UDP socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('127.0.0.1', self.port))
            self.sock.settimeout(2.0)
            
            # Start simulation
            self.sim.startSimulation()
            self.log("✓ Simulation started (free-running mode)")
            self.log(f"✓ Listening on UDP port {self.port}")
            
            self.packet_count = 0
            last_log_time = time.time()
            
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    positions_str = data.decode('utf-8').strip().split(',')
                    if len(positions_str) == 6:
                        positions = [float(x) for x in positions_str]
                        for j in range(6):
                            self.sim.setJointTargetPosition(self.joint_handles[j], positions[j])
                        self.packet_count += 1
                        
                        # Log every 2 seconds instead of every 20 packets
                        current_time = time.time()
                        if current_time - last_log_time >= 2.0:
                            self.log(f"  [{self.packet_count}] joints: {[round(p, 3) for p in positions]}")
                            last_log_time = current_time
                            
                except socket.timeout:
                    if self.packet_count > 0:
                        self.log(f"Stream paused after {self.packet_count} packets")
                        self.packet_count = 0
                    continue
                except Exception as e:
                    if self.running:  # Only log if we're still supposed to be running
                        self.log(f"Error in bridge loop: {e}")
                        
        except Exception as e:
            self.log(f"Fatal error in bridge: {e}")
        finally:
            if self.sock:
                self.sock.close()
            if self.sim:
                try:
                    self.sim.stopSimulation()
                except:
                    pass


def main():
    """Standalone bridge runner"""
    bridge = SimBridge(port=5000)
    
    if not bridge.start():
        print("Failed to start bridge")
        return 1
        
    print("Bridge running... Press Ctrl+C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping bridge...")
        bridge.stop()
        
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
