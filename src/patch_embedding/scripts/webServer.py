#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/5/11
    服务端，与后端（客户端）交互，调用主控函数
"""
import asyncio
import re

import websockets
import controller
#服务端ip地址、端口号
ip = '10.193.215.78'
port = 8765
#消息格式：和前后端url匹配，
#服务端响应函数
async def echo(websocket, path):
    async for message in websocket:
        print(message)
        if message=='map/create/':
            controller.getController().create_map_start()
            message = "I got your message: {}".format(message)
        elif re.match("map/save/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
                break
            controller.getController().create_map_save(int(result.group()))
            message = "I got your message: {}".format(message)
        elif re.match("mark/create/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
                break
            controller.getController().edit_mark(int(result.group()))
            message = "I got your message: {}".format(message)
        # elif re.match("mark/save:", message):

        else :
            message = "Invalid message!!!"
        await websocket.send(message)

# 注册服务端
asyncio.get_event_loop().run_until_complete(websockets.serve(echo, ip, port))
asyncio.get_event_loop().run_forever()