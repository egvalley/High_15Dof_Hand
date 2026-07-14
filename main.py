"""程序入口。"""

import tkinter as tk

from high_dof_gui import HighDofHandGUI
from sdk.config import AppConfig


def main():
    """
    程序入口：创建 Tk 根窗口、用默认 AppConfig 启动 GUI 并进入事件循环。

    用法：命令行执行 `python main.py`（见文件末尾的 __main__ 保护）。
    """
    root = tk.Tk()
    app = HighDofHandGUI(root, config=AppConfig())
    root.mainloop()


if __name__ == "__main__":
    main()
