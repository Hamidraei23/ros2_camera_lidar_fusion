#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk

GRID_SIZE = 10
CANVAS_SIZE = 500
CELL = CANVAS_SIZE // GRID_SIZE

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_gui')
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
    def publish_pose(self, x_idx: int, y_idx: int):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        # 1 cell = 1 meter by default
        ps.pose.position.x = float(y_idx)
        ps.pose.position.y = -float(x_idx)
        ps.pose.position.z = 1.0
        ps.pose.orientation.w = 1.0
        self.pub.publish(ps)
        self.get_logger().info(f'Published pose at ({x_idx}, {y_idx})')

class GridGUI:
    def __init__(self, node: PosePublisherNode):
        self.node = node

        self.x_idx = 0
        self.y_idx = 0
        self.prev = (0, 0)

        # Create main window
        self.root = tk.Tk()
        self.root.title("Pose Publisher GUI")

        # Canvas for 10×10 grid
        self.canvas = tk.Canvas(self.root,
                                width=CANVAS_SIZE,
                                height=CANVAS_SIZE,
                                bg='white')
        self.canvas.pack()

        # Publish button + Enter key
        self.btn = tk.Button(self.root,
                             text='Publish (Enter)',
                             command=self.on_publish)
        self.btn.pack(fill='x')
        self.root.bind('<Return>', lambda evt: self.on_publish())

        # Draw grid lines
        for i in range(1, GRID_SIZE):
            x = i * CELL
            self.canvas.create_line(x, 0, x, CANVAS_SIZE, fill='lightgray')
            self.canvas.create_line(0, x, CANVAS_SIZE, x, fill='lightgray')

        # 1) define the raw‐grid index of logical (0,0) → center
        self.origin_idx = (GRID_SIZE // 2, GRID_SIZE // 2)

        # 2) draw a small red cross at that center cell
        cx = (self.origin_idx[0] + 0.5) * CELL
        cy = CANVAS_SIZE - (self.origin_idx[1] + 0.5) * CELL
        s  = CELL * 0.2  # arm length = 20% of a cell
        self.canvas.create_line(cx - s, cy - s, cx + s, cy + s,
                                fill='red', width=2)
        self.canvas.create_line(cx - s, cy + s, cx + s, cy - s,
                                fill='red', width=2)

        # we no longer keep a moving “curr” – every arrow starts from center
        self.next = None
        self.next_cell = None

        self.arrow_id = None
        self.text_id  = None

        # bind clicks
        self.canvas.bind('<Button-1>', self.on_click)

        # ROS spin timer
        self._spin_timer()

    def on_click(self, event):
        ci = event.x // CELL
        cj = (CANVAS_SIZE - event.y) // CELL
        if not (0 <= ci < GRID_SIZE and 0 <= cj < GRID_SIZE):
            return
        self.next_cell = (ci, cj)
        # compute logical Δ = (ci, cj) – origin
        dx = ci - self.origin_idx[0]
        dy = cj - self.origin_idx[1]
        self.next = (dx, dy)
        self.redraw_arrow()

    def redraw_arrow(self):
        # remove old
        if self.arrow_id: self.canvas.delete(self.arrow_id)
        if self.text_id:  self.canvas.delete(self.text_id)
        if self.next_cell is None:
            return

        # start always at center cross
        cx = (self.origin_idx[0] + 0.5) * CELL
        cy = CANVAS_SIZE - (self.origin_idx[1] + 0.5) * CELL
        # end at clicked cell
        nx = (self.next_cell[0] + 0.5) * CELL
        ny = CANVAS_SIZE - (self.next_cell[1] + 0.5) * CELL

        # draw arrow
        self.arrow_id = self.canvas.create_line(cx, cy, nx, ny,
                                                arrow=tk.LAST, width=2, fill='red')
        # annotate Δ-vector
        dx, dy = self.next
        xm, ym = (cx + nx)/2, (cy + ny)/2 - 10
        self.text_id = self.canvas.create_text(
            xm, ym,
            text=f"Δ=({dx},{dy})",
            fill='blue',
            font=('Arial', 12, 'bold')
        )

    def on_publish(self):
        if self.next is None:
            return
        # publish the relative (dx,dy)
        x = self.next[0] + self.prev[0]
        y = self.next[1] + self.prev[1]
        self.node.publish_pose(x, y)
        self.prev = (x, y)
        # clear preview
        self.next = None
        self.next_cell = None
        if self.arrow_id:
            self.canvas.delete(self.arrow_id)
            self.arrow_id = None
        if self.text_id:
            self.canvas.delete(self.text_id)
            self.text_id = None

    def _spin_timer(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.root.after(50, self._spin_timer)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = PosePublisherNode()
    gui = GridGUI(node)
    gui.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
