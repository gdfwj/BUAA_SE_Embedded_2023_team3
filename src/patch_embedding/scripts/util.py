#!/usr/bin/env python
# coding=utf-8

import psutil
import os
import signal


def terminate_process(parent_pid):
    """结束进程树

    Args:
        parent_pid (int): 父进程id
    """
    process = psutil.Process(parent_pid)
    children = process.children(recursive=True)
    for child in children:
        os.kill(child.pid, signal.SIGTERM)