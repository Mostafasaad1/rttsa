import sys
import os
import subprocess
import socket
import math
import threading
import numpy as np
import io
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider,
    QLabel, QGroupBox, QFormLayout, QScrollArea, QPushButton, QTabWidget,
    QDoubleSpinBox, QSizePolicy, QTextEdit, QSplitter, QCheckBox
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QPixmap, QImage, QFont
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from mpl_toolkits.mplot3d import Axes3D

# Import the bridge module
try:
    from sim_bridge import SimBridge
    BRIDGE_AVAILABLE = True
except ImportError:
    BRIDGE_AVAILABLE = False
    print("Warning: sim_bridge module not available. Bridge integration disabled.")

class ScurveVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RT S-Curve Trajectory Controller")
        self.resize(1100, 700)

        # Default values for joint-space tab
        self.params = {
            "max_v": 10.0,
            "max_a": 5.0,
            "max_j": 20.0,
            "start_p": 0.0,
            "start_v": 5.0,
            "start_a": 0.0,
            "target_p": 50.0
        }
        
        # Bridge integration
        self.bridge = None
        self.bridge_enabled = False

        self.init_ui()
        self.apply_styles()
        self.update_plot()

    def apply_styles(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            QTabWidget::pane { border: 1px solid #333; background: #1a1a1a; }
            QTabBar::tab { background: #252525; color: #aaa; padding: 10px 20px; border: 1px solid #333; }
            QTabBar::tab:selected { background: #333; color: #00d4ff; border-bottom: 2px solid #00d4ff; }
            QGroupBox { border: 1px solid #333; border-radius: 6px; margin-top: 15px; font-weight: bold; color: #00d4ff; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #ccc; }
            QPushButton { background-color: #333; color: #eee; border: 1px solid #444; border-radius: 4px; padding: 6px; }
            QPushButton:hover { background-color: #444; }
            QPushButton#run_btn { background-color: #007acc; border: none; font-weight: bold; }
            QPushButton#run_btn:hover { background-color: #008ae6; }
            QDoubleSpinBox { background: #252525; color: #eee; border: 1px solid #444; border-radius: 3px; padding: 3px; }
            QScrollArea { border: none; background: transparent; }
            QScrollBar:vertical { border: none; background: #121212; width: 10px; }
            QScrollBar::handle:vertical { background: #333; min-height: 20px; border-radius: 5px; }
        """)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(10, 10, 10, 10)

        tabs = QTabWidget()
        root_layout.addWidget(tabs)

        # ── Tab 1: Joint-Space S-Curve ────────────────────────────────────────
        joint_tab = QWidget()
        tabs.addTab(joint_tab, "Joint-Space S-Curve")
        tab1_layout = QHBoxLayout(joint_tab)
        self.joint_splitter = QSplitter(Qt.Horizontal)

        # Control Panel
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(5, 5, 5, 5)

        # Constraints Group
        constraints_group = QGroupBox("Kinematic Constraints")
        constraints_layout = QFormLayout()
        
        self.sliders = {}
        
        self.add_slider(constraints_layout, "max_v", "Max Velocity", 1, 50, 10)
        self.add_slider(constraints_layout, "max_a", "Max Accel", 1, 20, 5)
        self.add_slider(constraints_layout, "max_j", "Max Jerk", 1, 100, 20)
        
        constraints_group.setLayout(constraints_layout)
        control_layout.addWidget(constraints_group)

        # Initial State Group
        state_group = QGroupBox("Initial State & Target")
        state_layout = QFormLayout()
        
        self.add_slider(state_layout, "start_p", "Start Pos", -50, 50, 0)
        self.add_slider(state_layout, "start_v", "Start Vel", -20, 20, 5)
        self.add_slider(state_layout, "start_a", "Start Accel", -10, 10, 0)
        self.add_slider(state_layout, "target_p", "Target Pos", -100, 100, 50)
        
        self.btn_send_udp = QPushButton("🚀 Send Target to Live Robot")
        self.btn_send_udp.setObjectName("run_btn")
        self.btn_send_udp.clicked.connect(self.send_udp_target)
        state_layout.addRow(self.btn_send_udp)
        
        state_group.setLayout(state_layout)
        control_layout.addWidget(state_group)
        
        # Bridge Control Group
        if BRIDGE_AVAILABLE:
            bridge_group = QGroupBox("CoppeliaSim Bridge")
            bridge_layout = QVBoxLayout()
            
            self.bridge_toggle = QCheckBox("Enable Integrated Bridge")
            self.bridge_toggle.setStyleSheet("QCheckBox { color: #ccc; }")
            self.bridge_toggle.stateChanged.connect(self.toggle_bridge)
            bridge_layout.addWidget(self.bridge_toggle)
            
            self.bridge_status = QLabel("Bridge: Disabled")
            self.bridge_status.setStyleSheet("color: #888; font-style: italic; font-size: 10px;")
            self.bridge_status.setWordWrap(True)
            bridge_layout.addWidget(self.bridge_status)
            
            bridge_group.setLayout(bridge_layout)
            control_layout.addWidget(bridge_group)
        
        control_layout.addStretch()

        # Plot Area
        self.plot_label = QLabel()
        self.plot_label.setAlignment(Qt.AlignCenter)
        self.plot_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.plot_label.setMinimumSize(400, 300)
        self.plot_label.setScaledContents(True)

        self.joint_splitter.addWidget(control_panel)
        self.joint_splitter.addWidget(self.plot_label)
        self.joint_splitter.setStretchFactor(1, 1)
        tab1_layout.addWidget(self.joint_splitter)

        # ── Tab 2: Cartesian Bezier Planner ───────────────────────────────────
        cart_tab = QWidget()
        tabs.addTab(cart_tab, "Cartesian Spline Planner")
        self._build_cartesian_tab(cart_tab)

    def toggle_bridge(self, state):
        """Toggle the integrated CoppeliaSim bridge"""
        if state and not self.bridge_enabled:
            # Start bridge
            self.bridge = SimBridge(port=5000, log_callback=self.bridge_log)
            if self.bridge.start():
                self.bridge_enabled = True
                self.bridge_status.setText("Bridge: ✓ Running on port 5000")
                self.bridge_status.setStyleSheet("color: #00ff88; font-style: italic; font-size: 10px;")
            else:
                self.bridge_toggle.setChecked(False)
                self.bridge_status.setText("Bridge: ✗ Failed to start")
                self.bridge_status.setStyleSheet("color: #ff4444; font-style: italic; font-size: 10px;")
        elif not state and self.bridge_enabled:
            # Stop bridge
            if self.bridge:
                self.bridge.stop()
                self.bridge = None
            self.bridge_enabled = False
            self.bridge_status.setText("Bridge: Disabled")
            self.bridge_status.setStyleSheet("color: #888; font-style: italic; font-size: 10px;")
    
    def bridge_log(self, message):
        """Callback for bridge log messages"""
        print(f"[Bridge] {message}")
        if hasattr(self, 'bridge_status'):
            # Update status with last message (truncated)
            short_msg = message[:50] + "..." if len(message) > 50 else message
            if self.bridge_enabled:
                self.bridge_status.setText(f"Bridge: {short_msg}")
    
    def send_udp_target(self):
        try:
            target = math.radians(self.params["target_p"])
            msg = f"{target:.4f},0.0,0.0,0.0,0.0,0.0"
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(msg.encode('utf-8'), ('127.0.0.1', 5001))
            sock.close()
            print(f"Sent live target to RT Robot (rad): {msg}")
        except Exception as e:
            print(f"Failed to send UDP: {e}")
    
    def closeEvent(self, event):
        """Clean up bridge on window close"""
        if self.bridge_enabled and self.bridge:
            self.bridge.stop()
        event.accept()

    def add_slider(self, layout, key, label_text, min_val, max_val, default_val):
        label = QLabel(str(default_val))
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * 10))
        slider.setMaximum(int(max_val * 10))
        slider.setValue(int(default_val * 10))
        
        slider.valueChanged.connect(lambda v: self.on_slider_change(key, v/10.0, label))
        
        layout.addRow(QLabel(label_text), label)
        layout.addRow(slider)
        self.sliders[key] = slider

    def on_slider_change(self, key, value, label):
        self.params[key] = value
        label.setText(f"{value:.1f}")
        self.update_plot()

    def update_plot(self):
        arm_runner = os.path.join("build", "tests", "arm_runner")
        if not os.path.exists(arm_runner):
            return

        target_rad = math.radians(self.params["target_p"])
        cmd = [arm_runner, str(target_rad), "0", "0", "0", "0", "0"]
        
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            data = np.genfromtxt("joint_trajectory.csv", delimiter=',', skip_header=1)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            if len(data) == 0 or data.shape[1] < 2:
                return

            fig = Figure(figsize=(7, 6), dpi=80, facecolor='#1a1a1a')
            FigureCanvasAgg(fig)
            
            ax_p = fig.add_subplot(3, 1, 1)
            ax_v = fig.add_subplot(3, 1, 2)
            ax_a = fig.add_subplot(3, 1, 3)
            axes = [ax_p, ax_v, ax_a]
            
            for ax in axes:
                ax.set_facecolor('#121212')
                ax.tick_params(colors='#aaa')
                ax.xaxis.label.set_color('#aaa')
                ax.yaxis.label.set_color('#aaa')
                ax.title.set_color('#00d4ff')
                ax.grid(True, alpha=0.1)

            ax_p, ax_v, ax_a = axes
            t = data[:, 0]
            pos = data[:, 1]
            
            dt = np.diff(t)
            dt = np.where(dt == 0, 1e-6, dt)
            vel = np.concatenate([[0], np.diff(pos) / dt])
            acc = np.concatenate([[0], np.diff(vel) / dt])

            ax_p.plot(t, pos, color='#00d4ff', linewidth=2)
            ax_p.set_ylabel("Position (rad)")
            ax_p.set_title(f"S-Curve Profile — Target: {self.params['target_p']:.1f}°")
            
            ax_v.plot(t, vel, color='#00ff88', linewidth=2)
            ax_v.set_ylabel("Velocity (rad/s)")
            
            ax_a.plot(t, acc, color='#ffaa00', linewidth=2)
            ax_a.set_ylabel("Accel (rad/s²)")
            ax_a.set_xlabel("Time (s)")
            
            fig.tight_layout()
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=100, facecolor=fig.get_facecolor())
            buf.seek(0)
            
            image = QImage.fromData(buf.getvalue())
            self.plot_label.setPixmap(QPixmap.fromImage(image))
        except Exception as e:
            print(f"Error updating plot: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # Cartesian Spline Planner Tab
    # ─────────────────────────────────────────────────────────────────────────

    def _build_cartesian_tab(self, parent):
        main_layout = QHBoxLayout(parent)
        self.cart_splitter = QSplitter(Qt.Horizontal)

        # ── Left: Input Panel ─────────────────────────────────────────────────
        input_panel = QWidget()
        input_layout = QVBoxLayout(input_panel)
        input_layout.setContentsMargins(5, 5, 5, 5)

        # Waypoints List
        wp_group = QGroupBox("Waypoints (Max 10)")
        wp_layout = QVBoxLayout()
        
        self.wp_list_layout = QVBoxLayout()
        self.wp_list_layout.setContentsMargins(5, 5, 5, 5)
        self.wp_list_layout.addStretch()
        
        self.waypoint_widgets = []
        
        btn_add_wp = QPushButton("➕ Add Waypoint")
        btn_add_wp.clicked.connect(self._add_waypoint)
        
        wp_layout.addLayout(self.wp_list_layout)
        wp_layout.addWidget(btn_add_wp)
        wp_group.setLayout(wp_layout)
        input_layout.addWidget(wp_group)

        # Global Options
        opt_grp = QGroupBox("Spline Options")
        opt_form = QFormLayout()
        self.arc_spin = QDoubleSpinBox()
        self.arc_spin.setRange(0.0, 0.5)
        self.arc_spin.setSingleStep(0.02)
        self.arc_spin.setValue(0.12)
        opt_form.addRow("Segment Arc (m):", self.arc_spin)
        self.arc_spin.valueChanged.connect(self._preview_bezier)
        opt_grp.setLayout(opt_form)
        input_layout.addWidget(opt_grp)

        # Execute button
        self.btn_bezier_run = QPushButton("🚀 Execute Multi-Point Path")
        self.btn_bezier_run.setObjectName("run_btn")
        self.btn_bezier_run.setMinimumHeight(40)
        self.btn_bezier_run.clicked.connect(self._run_bezier_ik)
        input_layout.addWidget(self.btn_bezier_run)

        # Status
        self.cart_status = QLabel("Ready")
        self.cart_status.setWordWrap(True)
        self.cart_status.setStyleSheet("color: #888; font-style: italic;")
        input_layout.addWidget(self.cart_status)

        input_layout.addStretch()
        self.cart_splitter.addWidget(input_panel)

        # ── Right side: 3D Plot + Side Views ─────────────────────────
        self.cart_plot_label = QLabel()
        self.cart_plot_label.setAlignment(Qt.AlignCenter)
        self.cart_plot_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.cart_plot_label.setMinimumSize(400, 300)
        self.cart_plot_label.setScaledContents(True)
        
        self.cart_splitter.addWidget(self.cart_plot_label)
        self.cart_splitter.setStretchFactor(1, 1)
        main_layout.addWidget(self.cart_splitter)

        # Pre-populate with 2 points after all UI elements are created
        self._add_waypoint(x=0.3, y=-0.3, z=0.3)
        self._add_waypoint(x=0.3, y=0.3, z=0.3)

    def _add_waypoint(self, x=0.3, y=0.0, z=0.3):
        if len(self.waypoint_widgets) >= 10:
            return
            
        idx = len(self.waypoint_widgets)
        wp_widget = QWidget()
        wp_layout = QHBoxLayout(wp_widget)
        wp_layout.setContentsMargins(2, 2, 2, 2)
        
        lbl = QLabel(f"P{idx}:")
        lbl.setFixedWidth(25)
        wp_layout.addWidget(lbl)
        
        spins = {}
        for ax, val in zip(["X", "Y", "Z"], [x, y, z]):
            sb = QDoubleSpinBox()
            sb.setRange(-2.0, 2.0)
            sb.setSingleStep(0.05)
            sb.setDecimals(3)
            sb.setValue(val)
            sb.valueChanged.connect(self._preview_bezier)
            wp_layout.addWidget(sb)
            spins[ax] = sb
            
        btn_del = QPushButton("×")
        btn_del.setFixedWidth(25)
        btn_del.setStyleSheet("color: #ff4444; font-weight: bold;")
        btn_del.clicked.connect(lambda: self._remove_waypoint(wp_widget))
        wp_layout.addWidget(btn_del)
        
        # Insert before the stretch
        self.wp_list_layout.insertWidget(self.wp_list_layout.count() - 1, wp_widget)
        self.waypoint_widgets.append({"widget": wp_widget, "spins": spins})
        self._preview_bezier()

    def _remove_waypoint(self, widget):
        if len(self.waypoint_widgets) <= 2:
            return # Keep at least 2 points
            
        for i, item in enumerate(self.waypoint_widgets):
            if item["widget"] == widget:
                self.waypoint_widgets.pop(i)
                widget.setParent(None)
                break
        
        # Update labels
        for i, item in enumerate(self.waypoint_widgets):
            item["widget"].layout().itemAt(0).widget().setText(f"P{i}:")
            
        self._preview_bezier()

    def _get_bezier_points(self, num_per_segment=40):
        """Compute Catmull-Rom to piecewise cubic Bezier points for smooth interpolation."""
        raw_wps = []
        for item in self.waypoint_widgets:
            s = item["spins"]
            raw_wps.append(np.array([s["X"].value(), s["Y"].value(), s["Z"].value()]))
        
        if len(raw_wps) < 2: return np.array([]), [], []
        
        arc = self.arc_spin.value()
        
        # 1. Insert via-points if arc > 0
        wps = []
        for i in range(len(raw_wps)):
            wps.append(raw_wps[i])
            if i < len(raw_wps) - 1 and arc > 0:
                mid = (raw_wps[i] + raw_wps[i+1]) / 2.0
                mid[2] += arc
                wps.append(mid)
                
        # 2. Compute Catmull-Rom tangents
        T = []
        for i in range(len(wps)):
            if i == 0:
                T.append(wps[1] - wps[0])
            elif i == len(wps) - 1:
                T.append(wps[-1] - wps[-2])
            else:
                T.append(0.5 * (wps[i+1] - wps[i-1]))
                
        # 3. Generate Bezier segments
        all_pts = []
        control_points = []
        
        for i in range(len(wps) - 1):
            p0 = wps[i]
            p1 = wps[i] + T[i] / 3.0
            p2 = wps[i+1] - T[i+1] / 3.0
            p3 = wps[i+1]
            control_points.append((p0, p1, p2, p3))
            
            ts = np.linspace(0, 1, num_per_segment)
            for t in ts:
                mt = 1 - t
                pt = mt**3*p0 + 3*mt**2*t*p1 + 3*mt*t**2*p2 + t**3*p3
                if len(all_pts) > 0 and t == 0: continue
                all_pts.append(pt)
                
        return np.array(all_pts), raw_wps, control_points

    def _preview_bezier(self):
        try:
            pts, wps, cps = self._get_bezier_points(30)
            if len(pts) == 0: return

            fig = Figure(figsize=(9, 6), dpi=90, facecolor='#1a1a2e')
            FigureCanvasAgg(fig)
            
            gs = fig.add_gridspec(3, 3, wspace=0.2, hspace=0.3)

            # Main 3D View
            ax_3d = fig.add_subplot(gs[:, 1:], projection='3d')
            ax_3d.set_facecolor('#0f0f23')
            ax_3d.tick_params(colors='#aaa', labelsize=7)
            ax_3d.set_title("Main 3D View", color='#00d4ff', fontsize=10)
            
            # Plot 3D Curve
            ax_3d.plot(pts[:, 0], pts[:, 1], pts[:, 2], color='#00d4ff', linewidth=2)
            
            # Control polygons in 3D
            for cp in cps:
                poly = np.array(cp)
                ax_3d.plot(poly[:, 0], poly[:, 1], poly[:, 2], color='#444', linestyle=':', alpha=0.3)
                
            # Waypoints in 3D
            wps_arr = np.array(wps)
            ax_3d.scatter(wps_arr[:, 0], wps_arr[:, 1], wps_arr[:, 2], color='#ff4444', s=50, zorder=5)
            for i, wp in enumerate(wps):
                ax_3d.text(wp[0], wp[1], wp[2], f" P{i}", color='#fff', fontsize=8)

            ax_3d.set_xlabel("X (m)", fontsize=7, color='#aaa')
            ax_3d.set_ylabel("Y (m)", fontsize=7, color='#aaa')
            ax_3d.set_zlabel("Z (m)", fontsize=7, color='#aaa')

            # Side views mapping
            views = [
                (0, 1, 'Top View (X-Y)', 'X', 'Y'),
                (0, 2, 'Front View (X-Z)', 'X', 'Z'),
                (1, 2, 'Side View (Y-Z)', 'Y', 'Z'),
            ]

            for row, (xi, yi, title, xl, yl) in enumerate(views):
                ax = fig.add_subplot(gs[row, 0])
                ax.set_facecolor('#0f0f23')
                ax.tick_params(colors='#aaa', labelsize=6)
                ax.set_title(title, color='#00d4ff', fontsize=8)
                ax.grid(True, alpha=0.1, color='#555')
                
                # Plot 2D projection
                ax.plot(pts[:, xi], pts[:, yi], color='#00d4ff', linewidth=1.5)
                ax.scatter(wps_arr[:, xi], wps_arr[:, yi], color='#ff4444', s=20, zorder=5)

            # No tight_layout needed since GridSpec handles it
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=90, facecolor=fig.get_facecolor(), bbox_inches='tight')
            buf.seek(0)
            self.cart_plot_label.setPixmap(QPixmap.fromImage(QImage.fromData(buf.getvalue())))
        except Exception as e:
            print("PREVIEW ERROR:", e); self.cart_status.setText(f"Preview error: {e}")

    def _run_bezier_ik(self):
        runner = os.path.join("build", "tests", "bezier_ik_runner")
        if not os.path.exists(runner):
            self.cart_status.setText("❌ Runner not found")
            return

        cmd = [runner]
        for item in self.waypoint_widgets:
            s = item["spins"]
            cmd.extend([str(s["X"].value()), str(s["Y"].value()), str(s["Z"].value())])
        
        cmd.append(str(self.arc_spin.value()))

        self.btn_bezier_run.setEnabled(False)
        self.cart_status.setText(f"⏳ Executing {len(self.waypoint_widgets)}-point path...")

        def worker():
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
                for line in proc.stdout:
                    print(line.rstrip()) # Log to console
                proc.wait()
                self.cart_status.setText("✅ Done" if proc.returncode == 0 else "❌ Failed")
            except Exception as e:
                print(f"Error: {e}")
                self.cart_status.setText("❌ Error")
            finally:
                self.btn_bezier_run.setEnabled(True)

        threading.Thread(target=worker, daemon=True).start()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ScurveVisualizer()
    window.show()
    sys.exit(app.exec())
