from PyQt6.QtWidgets import QTabWidget
from tabs.velocity_tab import VelocityTab
from tabs.pose_tab import PoseTab
from tabs.sequence_tab import SequenceTab
from tabs.joint_tab import JointTab
from tabs.plot_tab import PlotTab

class ControlTabs(QTabWidget):
    def __init__(self, main_win):
        super().__init__()
        self.pose_tab = PoseTab(main_win)
        self.seq_tab = SequenceTab(main_win)
        self.joint_tab = JointTab(main_win)
        self.vel_tab = VelocityTab(main_win)
        self.plot_tab = PlotTab(main_win)

        self.addTab(self.pose_tab, "Cartesian")
        self.addTab(self.seq_tab, "Sequence")
        self.addTab(self.joint_tab, "Joint Jog")
        self.addTab(self.vel_tab, "EE VELOCITY")
        self.addTab(self.plot_tab, "Plot EE")