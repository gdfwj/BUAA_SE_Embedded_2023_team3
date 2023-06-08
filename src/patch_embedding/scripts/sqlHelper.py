# import pymysql
from django.db import connection

class SqlHelper():
    def __init__(self):
        """
        创建一个SqlHelper
        """
        try:
            self.con = connection
            # self.con.autocommit(1)
            self.cursor = self.con.cursor()
            # self.con.select_db('databaseHW')
        except:
            print("DataBase connect error,please check the db config.")

    def executeProcedure(self, pname, params:list):
        """
        执行存储过程\n
        :param pname: 存储过程名
        :param params: 存储过程需要的参数
        :return:
        """
        try:
            print(pname, params)
            self.cursor.callproc(pname, params)
            records = self.cursor.fetchall()
            return records
        except Exception as e:
            print(e)
            raise e

    def isExistTable(self, tableName):
        sql = "select * from %s" % tableName
        result = self.executeSql(sql)
        if result is None:
            return False
        else:
            return True

    def update(self, tablename, attrs_dict, cond_dict):
        """更新数据
            args:
                tablename: 表名字
                attrs_dict: 更新属性键值对
                cond_dict:  更新条件
        """
        attrs_list = []
        consql = " "
        for k, v in attrs_dict.items():
            #表的属性名不必加引号, 但是属性值需要加
            attrs_list.append(" " + k + " =" + "\'" + str(v) + "\'")
        attrs_sql = ",".join(attrs_list)
        if cond_dict is not None:
            for k, v in cond_dict.items():
                if isinstance(v, str):
                    v = "\'" + v + "\'"
                consql += k + "=" + str(v) + ' and '
        else :
                print("update error because cond_dict is None！")
                return
        consql = consql[:-4]
        sql = "update %s set %s where %s" % (tablename, attrs_sql, consql)
        print(sql)
        self.executeCommit(sql)

    def delete(self, tablename, cond_dict=None):
        """删除数据
        """
        sql = ' '
        if cond_dict is not None:
            for k,v in cond_dict.items():
                if isinstance(v, str):
                    v = "\'" + v + "\'"
                sql += k + "=" + str(v) + ' and '
        else :
            sql = "delete from %s" %(tablename)
            print(sql)
            self.executeCommit(sql)
            return
        sql = sql[:-4]
        sql = "delete from %s where %s" %(tablename, sql)
        print(sql)
        self.executeCommit(sql)

    def insert(self, table, params_dict:dict):
        """插入数据
        :param table: 表名
        :param params_dict: {属性列名:常量}
        :return:
        """
        key = []
        value = []
        for k, v in params_dict.items():
            key.append(k)
            if isinstance(v, str):
                value.append('\'' + v + '\'')
            else :
                value.append(str(v))
        sql = 'insert into %s' % table
        sql += '(' + ','.join(key) + ')' + ' values(' + ','.join(value) + ')'
        print('insert:' + sql)
        self.executeCommit(sql)

    def select(self, table, listnames=None, cond_dict=None, all=False):
        """
        分有三种情况:
            1. all为True, 其余为None, 此时查询全表内容
            2. all为True, cond_dict不为None, 此时查询符合条件的所有列
            3. all为false, 则按列和条件查找
        :param table:
        :param listnames:
        :param cond_dict:
        :param all:
        :return:
        """
        if all and (cond_dict is None):
            sql = 'select * from ' + table
            return self.executeSql(sql)
        elif all:
            sql = 'select * from ' + table
        else :
            sql = 'select ' + ','.join(listnames) + ' from ' + table#涉及到表属性名的不加引号
        consql = ""
        if cond_dict is not None:
            consql = " where "
            for k, v in cond_dict.items():
                if isinstance(v, str):
                    v = "\'" + v + "\'"
                consql += k + "=" + str(v) + ' and '
            consql = consql[:-4]
        sql += consql
        print(sql)
        return self.executeSql(sql)

    def executeSql(self, sql):
        """
        Executes a mysql statement that queries something like *SELECT*
        :param sql: a mysql statement
        :return: a result set for a read operation
        """
        try:
            self.cursor.execute(sql)
            records = self.cursor.fetchall()
            return records
        except Exception as e:
            error = '执行sql语句失败(%s): %s' % (e.args[0], e.args[1])
            putError(error)
            raise e

    def executeCommit(self, sql):
        """
        执行sql语句，针对更新，删除等。操作失败时回滚
        :param sql: sql语句
        :return: void
        """
        try:
            self.cursor.execute(sql)
            self.con.commit()
        except Exception as e:
            self.con.rollback()
            error = '执行数据库sql语句失败(%s): %s' % (e.args[0], e.args[1])
            putError(error)
            raise error

    # def closeDataBase(self):
    #     """
    #     关闭数据库连接
    #     :return: void
    #     """
    #     if self.con:
    #         self.con.close()
    #     else:
    #         print("DataBase doesn't connect,close connectiong error;please check the db config.")
    #

def putError(e):
    """输出红色错误信息
    """
    err = "Error:"
    print("\033[0;31;40m", err, e, "\033[0m")