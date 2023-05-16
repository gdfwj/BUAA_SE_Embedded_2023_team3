<template>
  <el-container direction="vertical" class="homepage">
<!--    头部-->
    <el-header style="padding-right: 0; position: relative!important;">
      <el-row :gutter="20" type="flex">
        <el-col :span="4" :offset="0">
          <div style="line-height:60px;height:100%">
            家庭服务机器人
          </div>
        </el-col>
        <el-col :span="12"></el-col>
        <el-col :span="8" >
          <el-menu
            :default-active="activeIndex"
            class="el-menu-demo"
            mode="horizontal"
            @select="handleSelect">
          <el-menu-item index="1">
            <template>
              <i class="el-icon-s-home"></i>
              <span>首页</span>
            </template>
          </el-menu-item>
          <el-menu-item index="2">
            <template><i class="el-icon-question">
            </i><span>帮助</span>
            </template>
          </el-menu-item>
          <el-menu-item index="3">
            <template>
              <i class="el-icon-info">
              </i><span>关于我们</span>
            </template>
          </el-menu-item>
          <el-submenu index="4">
            <template slot="title">
              <i class="el-icon-s-tools"></i>
              <span>设置</span></template>
            <el-menu-item index="4" @click="updateSys">
              <span>系统升级</span>
            </el-menu-item>
            <el-menu-item index="4" @click="resetFlag = true">
              <span>恢复出厂设置</span>
            </el-menu-item>
          </el-submenu>
          <el-dialog
            title="确定要恢复出厂设置吗"
            :visible.sync="resetFlag"
            width="30%"
            :before-close="handleClose">
            <span slot="footer" class="dialog-footer">
              <el-button type="primary" @click="resetFlag = false">取 消</el-button>
              <el-button type="danger" @click="resetSys">确定</el-button>
            </span>
          </el-dialog>
        </el-menu></el-col>
      </el-row>
    </el-header>
    <el-main>
      <mainPage v-if="activeIndex == '1'"></mainPage>
      <help v-if="activeIndex == '2'"></help>
      <about v-if="activeIndex == '3'"></about>
    </el-main>
  </el-container>
</template>

<script>
/* eslint-disable*/
import mainPage from './InnerPages/MainPage.vue';
import about from '../components/About';
import help from '../components/Help';

export default {
  name: 'Home',
  data() {
    return {
      activeIndex: '1',
      resetFlag:false,
    }
  },
  methods: {
    handleSelect(key) {
      this.activeIndex = key;
    },
    handleClose(done) {
      this.$confirm('确认关闭？')
        .then(() => {
          done();
        })
        .catch(() => {});
    },
    resetSys() {
      this.$axios.post(`http://localhost:8000/main/resetAll/`)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.$message.success("成功初始化系统设置！")
            this.resetFlag = false;
            this.activeIndex = '1';
          }
        })
    },
    updateSys() {
      this.$axios.post(`http://localhost:8000/main/update/`)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.$message.success("系统升级成功")
          }
        })
    }
  },
  components: {
    mainPage,
    help,
    about,
  }
}
</script>

<style scoped>

</style>
