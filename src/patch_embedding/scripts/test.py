class Demo(object):
    a = 100
    def fun1(self):
        print("fun1")
    def fun2(self):
        print("fun2")
    def fun3(self):
        #在类中调用另一个函数
        getattr(self,"fun2")()


demo = Demo()
#调用类中的函数
getattr(demo,"fun1")()#fun1
getattr(demo,"fun3")()#fun2
#获取类中的属性值
print(getattr(demo,"a"))#100
#调用一个不存在的属性
print(getattr(demo,"b"))
#对于不存在的属性给一个默认值，避免抛出异常
print(getattr(demo,"b",10))#10
