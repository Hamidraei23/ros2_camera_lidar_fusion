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

        # simple flags instead of topics
        self.hold = False
        self.land = False
        self.emergency = False

        # altitude delta from slider
        self.altitude_std = 0

    def publish_pose(self, x_idx: int, y_idx: int):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()

        # choose X behavior based on flags

        ps.pose.position.x = float(y_idx)


        if self.hold:
            ps.pose.position.x = -1000.0
        elif self.land:
            ps.pose.position.x = -2000.0
        elif self.emergency:
            ps.pose.position.x = -3000.0
        
        self.altitude_std += float(self.altitude_delt)

        ps.pose.position.y = -float(x_idx)

        ps.pose.position.z = self.altitude_std +1
        ps.pose.orientation.w = 1.0

        self.pub.publish(ps)
        self.get_logger().info(
            f'Published pose at ({-x_idx}, {y_idx}), Δalt={self.altitude_delt}, '
            f'hold={self.hold}, land={self.land}, emergency={self.emergency}'
        )

    # these just flip the flags
    def set_hold(self):
        self.hold = True
        self.land = False
        self.emergency = False
        self.get_logger().info('HOLD flag set')

    def set_land(self):
        self.hold = False
        self.land = True
        self.emergency = False
        self.get_logger().info('LAND flag set')

    def set_emergency(self):
        self.hold = False
        self.land = False
        self.emergency = True
        self.get_logger().info('EMERGENCY flag set')


class GridGUI:
    def __init__(self, node: PosePublisherNode):
        self.node = node
        self.prev = (0, 0)
        self.next = None
        self.next_cell = None
        self.arrow_id = None
        self.text_id  = None

        # --- TK setup ---
        self.root = tk.Tk()
        self.root.title("Pose Publisher GUI")

        # Slider for altitude delta
        self.alt_slider = tk.Scale(
            self.root,
            from_=10, to=-10,
            resolution=1,
            orient=tk.VERTICAL,
            label='Δ Altitude'
        )
        self.alt_slider.set(0)
        self.alt_slider.pack(side='right', fill='y', padx=5, pady=5)

        # Canvas
        self.canvas = tk.Canvas(
            self.root,
            width=CANVAS_SIZE,
            height=CANVAS_SIZE,
            bg='white'
        )
        self.canvas.pack(side='left')

        # Button row
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill='x', pady=5)
        tk.Button(btn_frame, text='Publish (Enter)',
                  command=self.on_publish).pack(side='left', expand=True, fill='x')
        tk.Button(btn_frame, text='Hold',
                  command=self.on_hold).pack(side='left', expand=True, fill='x')
        tk.Button(btn_frame, text='Land',
                  command=self.on_land).pack(side='left', expand=True, fill='x')
        tk.Button(btn_frame, text='Emergency',
                  fg='white', bg='red',
                  command=self.on_emergency).pack(side='left', expand=True, fill='x')
        self.root.bind('<Return>', lambda evt: self.on_publish())

        # Draw grid + center cross
        for i in range(1, GRID_SIZE):
            x = i * CELL
            self.canvas.create_line(x, 0, x, CANVAS_SIZE, fill='lightgray')
            self.canvas.create_line(0, x, CANVAS_SIZE, x, fill='lightgray')

        self.origin_idx = (GRID_SIZE // 2, GRID_SIZE // 2)
        cx = (self.origin_idx[0] + 0.5) * CELL
        cy = CANVAS_SIZE - (self.origin_idx[1] + 0.5) * CELL
        s  = CELL * 0.2
        self.canvas.create_line(cx - s, cy - s, cx + s, cy + s, fill='red', width=2)
        self.canvas.create_line(cx - s, cy + s, cx + s, cy - s, fill='red', width=2)

        self.canvas.bind('<Button-1>', self.on_click)
        self._spin_timer()

    def on_click(self, event):
        ci = event.x // CELL
        cj = (CANVAS_SIZE - event.y) // CELL
        if not (0 <= ci < GRID_SIZE and 0 <= cj < GRID_SIZE):
            return
        self.next_cell = (ci, cj)
        dx = ci - self.origin_idx[0]
        dy = cj - self.origin_idx[1]
        self.next = (dx, dy)
        self.redraw_arrow()

    def redraw_arrow(self):
        if self.arrow_id: self.canvas.delete(self.arrow_id)
        if self.text_id:  self.canvas.delete(self.text_id)
        if self.next_cell is None:
            return
        cx = (self.origin_idx[0] + 0.5) * CELL
        cy = CANVAS_SIZE - (self.origin_idx[1] + 0.5) * CELL
        nx = (self.next_cell[0] + 0.5) * CELL
        ny = CANVAS_SIZE - (self.next_cell[1] + 0.5) * CELL
        self.arrow_id = self.canvas.create_line(
            cx, cy, nx, ny, arrow=tk.LAST, width=2, fill='red'
        )
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

        # read slider into node.altitude_delt, then reset slider
        self.node.altitude_delt = self.alt_slider.get()
        self.alt_slider.set(0)

        x = self.next[0] + self.prev[0]
        y = self.next[1] + self.prev[1]
        self.node.publish_pose(x, y)
        self.prev = (x, y)
        self.next = None
        self.next_cell = None
        if self.arrow_id:
            self.canvas.delete(self.arrow_id)
            self.arrow_id = None
        if self.text_id:
            self.canvas.delete(self.text_id)
            self.text_id = None

    def on_hold(self):
        self.node.set_hold()

    def on_land(self):
        self.node.set_land()

    def on_emergency(self):
        self.node.set_emergency()

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
