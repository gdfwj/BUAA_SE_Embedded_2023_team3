<template>
<!--  主页面  包含build 和 serve-->
  <div class="component-content">
    <el-row>
      <el-col :span="24" :offset="0" class="big center" >
        <span>服务模式选择</span>
      </el-col>
    </el-row>
    <el-row class="space"></el-row>
    <el-row>
      <el-col :span="24" :offset="0" class="small center">
        <span>SERVICE MODE SELECTION</span>
      </el-col>
    </el-row>
    <el-row class="space"></el-row>
    <el-row class="space"></el-row>
    <el-row class="space"></el-row>
    <el-row>
      <el-col :span="24" :offset="0" class="small center">
        <span>请根据需求选择以下两种模式中的一个</span>
      </el-col>
    </el-row>
    <el-row class="space"></el-row>
    <el-row class="space"></el-row>
    <el-row :gutter="100">
      <el-col :span="12" :offset="0">
        <el-button round class="center back-white button-box b-shadow" @click="gotoSet">
          <div class="image-box">
            <el-image :src="src_set">
              <div slot="placeholder" class="image-slot">
                <i class="el-icon-loading"></i>
              </div>
              <div slot="error" class="image-slot">
                <i class="el-icon-picture-outline"></i>
              </div>
            </el-image>
          </div>
          <div class="space"></div>
          <div class="item-box middle">
            <span>配置模式</span>
          </div>
          <div class="space"></div>
          <div class="item-box small gray">
            <span>地图和航点配置</span>
          </div>
          <div class="space"></div>
        </el-button>
      </el-col>
      <el-col :span="12" :offset="0">
        <el-button round class="center back-white button-box b-shadow" @click="gotoServe">
          <div class="image-box">
            <el-image :src="src_serve">
              <div slot="placeholder" class="image-slot">
                <i class="el-icon-loading"></i>
              </div>
              <div slot="error" class="image-slot">
                <i class="el-icon-picture-outline"></i>
              </div>
            </el-image>
          </div>
          <div class="space"></div>
          <div class="item-box middle">
            <span>服务模式</span>
          </div>
          <div class="space"></div>
          <div class="item-box small gray">
            <span>开始服务</span>
          </div>
          <div class="space"></div>
        </el-button>
      </el-col>
    </el-row>
  </div>
</template>

<script>
/* eslint-disable*/
export default {
  name: 'MainPage',
  data() {
    return {
      src_set: require('../../assets/配置.png'),
      src_serve: require('../../assets/服务.png'),
    }
  },
  methods: {
    gotoSet() {
      this.$router.push({
        name: "Build"
      })
    },
    gotoServe() {
      this.$router.push({
        name: "Serve"
      })
    }
  }
}
</script>

<style scoped>
.center {
  text-align: center;
}
.small {
  font-size: 14px;
}
.middle {
  font-size: 20px;
}
.big {
  font-size: 28px;
}
.blue {
  color: #034DFF;
}
.gray {
  color: gray;
}
.space {
  height: 20px;
}
.component-content {
  padding-left: 20%;
  padding-right: 20%;
  height: 100%;
}
.back-white {
  background-color: rgb(255, 255, 255)!important;
}
.item-box {
  min-height: 30px;
}
.image-box {
  min-height: 60px;
  padding-top: 30px;
  padding-bottom: 30px;
}
.button-box {
  width: 100% !important;
}
.b-shadow {
  box-shadow: 10px 10px 5px grey!important;
}
</style>
