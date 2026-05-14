import tkinter as tk
import subprocess
import threading


class ShowcaseUI:

    def __init__(self, root):

        self.root = root

        root.title("Autonomous Delivery Robot")
        root.geometry("900x500")
        root.configure(bg="black")

        self.title = tk.Label(
            root,
            text="AUTONOMOUS DELIVERY ROBOT",
            font=("Arial", 30, "bold"),
            fg="white",
            bg="black"
        )

        self.title.pack(pady=30)

        self.status = tk.Label(
            root,
            text="SYSTEM OFFLINE",
            font=("Arial", 24),
            fg="red",
            bg="black"
        )

        self.status.pack(pady=20)

        self.info = tk.Label(
            root,
            text="ROS2 + LiDAR + AI Vision + Semantic Navigation",
            font=("Arial", 16),
            fg="white",
            bg="black"
        )

        self.info.pack(pady=10)

        # =========================
        # Buttons
        # =========================

        self.bringup_button = tk.Button(
            root,
            text="START SYSTEM",
            font=("Arial", 18, "bold"),
            width=20,
            height=2,
            bg="green",
            fg="white",
            command=self.start_system
        )

        self.bringup_button.pack(pady=20)

        self.behavior_button = tk.Button(
            root,
            text="START AUTONOMY",
            font=("Arial", 18, "bold"),
            width=20,
            height=2,
            bg="blue",
            fg="white",
            command=self.start_behavior
        )

        self.behavior_button.pack(pady=20)

        self.stop_button = tk.Button(
            root,
            text="STOP AUTONOMY",
            font=("Arial", 18, "bold"),
            width=20,
            height=2,
            bg="red",
            fg="white",
            command=self.stop_behavior
        )

        self.stop_button.pack(pady=20)

        self.check_nodes()

    # =========================
    # Start system bringup
    # =========================

    def start_system(self):

        self.status.config(
            text="LOADING AI SYSTEM...",
            fg="yellow"
        )

        cmd = """
        cd ~/robot_ws;
        source /opt/ros/humble/setup.bash;
        source install/setup.bash;
        ros2 launch robot_mvp system_bringup.launch.py
        """

        threading.Thread(
            target=lambda: subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", cmd]
            )
        ).start()

    # =========================
    # Start autonomy
    # =========================

    def start_behavior(self):

        cmd = """
        cd ~/robot_ws;
        source /opt/ros/humble/setup.bash;
        source install/setup.bash;
        ros2 launch robot_mvp showcase_behavior.launch.py
        """

        threading.Thread(
            target=lambda: subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", cmd]
            )
        ).start()

    # =========================
    # Stop autonomy
    # =========================

    def stop_behavior(self):

        subprocess.call(
            "pkill -f showcase_behavior_node",
            shell=True
        )

    # =========================
    # Node status check
    # =========================

    def check_nodes(self):

        result = subprocess.getoutput("ros2 node list")

        if (
            "person_detection_node" in result and
            "semantic_decision_node" in result and
            "lidar_safety_node" in result
        ):

            self.status.config(
                text="SYSTEM READY",
                fg="lime"
            )

        self.root.after(1000, self.check_nodes)


root = tk.Tk()

app = ShowcaseUI(root)

root.mainloop()