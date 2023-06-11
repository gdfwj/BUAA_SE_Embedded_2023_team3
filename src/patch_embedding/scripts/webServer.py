#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/5/11
    服务端，与后端（客户端）交互，调用主控函数
"""
import asyncio
import re
import rospy
from patch_embedding.srv import Conn

import websockets
import signal
import sys

#服务端ip地址、端口号
ip = '10.193.215.78'
# ip = 'localhost'
port = 8765

#消息格式：和前后端url匹配，
#服务端响应函数
async def echo(websocket, path):
    async for message in websocket:
        print(message)
        if message=='map/create/':
            resp = client("create_map_start", 0, "")
            message = "I got your message: {}".format(message)
        elif re.match("map/save/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                resp = client("create_map_save", int(result.group()), "")
                message = "I got your message: {}".format(message)
        elif re.match("mark/create/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                resp = client("edit_mark", int(result.group()), "")
                message = "I got your message: {}".format(message)
        elif re.match("mark/save/:", message):
            map_id = message.split(":")[1]
            label_id = message.split(":")[2]
            if map_id == None or label_id == None:
                message = "Wrong, please send id"
            else :
                resp = client("save_mark", int(map_id), str(label_id))
                message = "I got your message: {}".format(message)
        elif message == 'object/fetch/':
            resp = client("grab", 0, "")
            message = "I got your message: {}".format(message)
        elif re.match("service/init/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                resp = client("navigation_init", int(result.group()), "")
                message = "I got your message: {}".format(message)
        elif re.match("navigation/begin/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                resp = client("navigation_begin", int(result.group()), "")
                message = "I got your message: {}".format(message)
        else :
            message = "Invalid message!!!"
        await websocket.send(message)

def quit(signum, frame):
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node("webServer")
    client = rospy.ServiceProxy('/control/web', Conn)
    rospy.wait_for_service('/control/web')
    # 实现Ctrl+C退出程序
    signal.signal(signal.SIGINT, quit)

    # 注册服务端
    asyncio.get_event_loop().run_until_complete(websockets.serve(echo, ip, port))
    asyncio.get_event_loop().run_forever()
