"""程序入口。"""

import tkinter as tk

from high_dof_gui import HighDofHandGUI
from sdk.config import AppConfig


def main():
    root = tk.Tk()
    app = HighDofHandGUI(root, config=AppConfig())
    root.mainloop()


if __name__ == "__main__":
    main()
