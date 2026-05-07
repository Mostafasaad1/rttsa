"""
Standalone CoppeliaSim Bridge Runner
Uses the sim_bridge module for consistency with GUI integration
"""
import sys
from sim_bridge import SimBridge

def main():
    """Run the bridge standalone"""
    print("=" * 60)
    print("RT Trajectory Smoothing - CoppeliaSim Bridge")
    print("=" * 60)
    print()
    
    bridge = SimBridge(port=5000)
    
    if not bridge.start():
        print("\n✗ Failed to start bridge")
        print("\nTroubleshooting:")
        print("  1. Ensure CoppeliaSim is running")
        print("  2. Check that the UR5 robot is in the scene")
        print("  3. Verify coppeliasim_zmqremoteapi_client is installed:")
        print("     pip install coppeliasim-zmqremoteapi-client")
        return 1
    
    print()
    print("=" * 60)
    print("Bridge is running. Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        while True:
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        bridge.stop()
        print("✓ Bridge stopped cleanly")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
