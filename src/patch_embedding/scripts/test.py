a = None

class A:
    def __init__(self):
        pass

def fun():
    print(type(a))

if __name__ == '__main__':
    a = A()
    fun()