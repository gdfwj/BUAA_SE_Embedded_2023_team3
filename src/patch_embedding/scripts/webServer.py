#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/5/11
    服务端，与后端（客户端）交互，调用主控函数
"""
import asyncio
import websockets
import controller
#服务端ip地址、端口号
ip = 'localhost'
port = 8765
#消息格式：和前后端url匹配，
#服务端响应函数
async def echo(websocket, path):
    async for message in websocket:
        if message=='map/create/':
            print(message)
            controller.getController().create_map_start()
            message = "I got your message: {}".format(message)
        await websocket.send(message)

# 注册服务端
asyncio.get_event_loop().run_until_complete(websockets.serve(echo, ip, port))
asyncio.get_event_loop().run_forever()