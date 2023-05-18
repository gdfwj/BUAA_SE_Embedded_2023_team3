<!-- eslint-disable -->
<template>
  <el-container>
    <el-header>
      <el-row :gutter="20" style="padding:0; margin: 10px 0 0 0">
        <el-col :span="6">
          <el-page-header @back="goBack" content="服务模式">
          </el-page-header>
        </el-col>
        <el-col :span="14">
          <space></space>
        </el-col>
        <el-col :span="7" style="float: right; align-content: center;text-align: center; padding-right: 5%">
          <el-switch
            style="display: block"
            v-model="switchValue"
            active-color="#13ce66"
            inactive-color="#FFABAB"
            active-text="人工标识"
            inactive-text="语言控制"
            @click.native="changeVoice">
          </el-switch>
        </el-col>
      </el-row>
    </el-header>

    <el-main>
      <!--    地图部分-->
      <el-row>
        <el-col :span="7" :gutter="20" v-for="(map, index) in maps" :key="map.map_id" style="padding-left: 4%">
<!--          地图card-->
          <el-card
            shadow="hover"
            class="card"
            style="width: 100%; margin-left: 5%;
            padding-left: 5%; margin-bottom: 6px; margin-right: 5%"
            @click.native="getServe(map.map_id)">
            <el-row>
<!--             地图图片-->
              <el-col>
                <el-image :src="require(`/home/jinghongbin/SE/team03-project/src/patch_embedding/maps/map`+ map.map_id + `.png`)" style="width: 100%"></el-image>
              </el-col>
            </el-row>
            <el-row style="padding-top: 10px">
              <!--              地图名称、时间-->
              <el-col style="text-align: center">
                <el-tag size="small" style="align-content: center">{{map.map_name}}</el-tag>
                <el-tag size="small" style="align-content: center">{{map.map_time}}</el-tag>
              </el-col>
            </el-row>
            <!--            地图备注-->
            <el-row style="padding-top: 10px">
              <el-col style="text-align: center">
                <el-tag type="info" style="align-content: center">{{map.map_remark}}</el-tag>
              </el-col>
            </el-row>
          </el-card>
        </el-col>
      </el-row>

<!--      具体服务操作-->
      <el-dialog title="执行服务" :visible.sync="dialogServe" width="60%">
<!--        地图显示-->
        <el-row style="padding:5px; margin: 5px">
          <el-image
            style="width: 50%; height: 50%; padding-left: 25%"
            :src="require(`/home/jinghongbin/SE/team03-project/src/patch_embedding/maps/map`+ map.map_id + `.png`)">
          </el-image>
        </el-row>
        <!--          手动标识-->
        <div v-if="switchValue == true">
          <!--        功能选择-->
          <el-row  :gutter="0">
            <el-col :span="4" style="margin-top: 10px;padding-left: 50px">
              <b>功能选择:</b>
            </el-col>
            <el-col :span="7" style="padding-left: 0">
              <el-select v-model="value" placeholder="请选择功能" >
                <el-option
                  v-for="item in meNuOptions"
                  :key="item.value"
                  :label="item.label"
                  :value="item.value">
                </el-option>
              </el-select>
            </el-col>
          </el-row>
    <!--        导航-->
          <div v-if="value == 'label1'" style="text-align: center; align-content: center; align-items: center">

            <el-row style="padding-top: 20px">
              <el-col :span="4" style="margin-top: 3px;padding-left: 30px">
                结束位置：
              </el-col>
              <el-col :span="4">
                <el-select
                  v-model="navEndPoint"
                  placeholder="结束位置"
                 >
                  <el-option
                    v-for="item in pointsOptions"
                    :key="item.value"
                    :label="item.label"
                    :value="item.value">
                  </el-option>
                </el-select>
              </el-col>
              <el-col :span="4" style="float: right">
                <el-button type="success" @click="submitNav">开始导航</el-button>
              </el-col>
            </el-row>
          </div>
<!--          抓取-->
          <div v-if="value == 'label2'" style="text-align: center; align-content: center; align-items: center">
              <el-row style="padding-top: 20px">
                <el-col :span="4" style="margin-top: 3px;padding-left: 30px">
                  目标位置：
                </el-col>
                <el-col :span="4">
                  <el-select
                    v-model="catchEndPoint"
                    placeholder="结束位置"
                    >
                    <el-option
                      v-for="item in pointsOptions"
                      :key="item.value"
                      :label="item.label"
                      :value="item.value">
                    </el-option>
                  </el-select>
                </el-col>
                <el-col :span="4" style="float: right">
                  <el-button type="warning" @click="submitFetch">开始取物</el-button>
                </el-col>
              </el-row>
          </div>
        </div>
        <!--          语音控制-->
        <div v-if="switchValue == false">
          <el-row style="padding-top: 20px; padding-left: 5%; padding-right: 5%">
            <el-input placeholder="正在说话中..." v-model="inputString" class="input-with-select">
              <el-button slot="append" icon="el-icon-check"></el-button>
            </el-input>
          </el-row>
        </div>

      </el-dialog>
    </el-main>
  </el-container>
</template>

<script>
/* eslint-disable */

export default {
  name: 'Serve',
  data() {
    return {
      maps:[],
      inputString: '',
      dialogServe: false,
      map_id: '1',
      value: '',
      switchValue: true,
      meNuOptions: [
        {
          value: 'label1',
          label: '导航控制'
        },
        {
          value: 'label2',
          label: '取物控制'
        }
      ],
      pointsOptions:[],
      navEndPoint: '请选择结束位置',
      catchEndPoint: '请选择结束位置'
    }
  },
  created() {
    this.maps = this.getMaps()
  },
  methods: {
    goBack() {
      this.$router.push({
        name: 'Home'
      })
    },
    // get marks
    getMarks(map_id) {
      let x = {"map_id": map_id}
      console.log(x)
      this.$axios.post(`http://localhost:8000/mark/show/`, x)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
          this.pointsOptions = []
            for (var i = 0; i < res.data.data.length; i+= 1) {
              this.pointsOptions.push(
                { label : res.data.data[i].label_name,
                  value: res.data.data[i].label_id}
              )
            }
          }
        })
      /*
      var datas = [
        {
          label_id: 1,
          label_name: '航点1',
          label_remark: 'hello'
        },
        {
          label_id: 2,
          label_name: '航点2',
          label_remark: 'hello 2'
        },
      ]
      this.pointsOptions = []
      for (var i = 0; i < datas.length; i+= 1) {
        this.pointsOptions.push(
          { label : datas[i].label_name,
            value: String(datas[i].label_id)}
        )
      }
       */
    },
    getSRC(url) {
      return url
    },
    getServe(map_id) {
      this.dialogServe = true
      this.map_id = map_id
      this.getMarks(map_id)
      this.initService(map_id)
    },
    // 初始化服务
    initService(map_id) {
      let x = {map_id: map_id}
      this.$axios.post(`http://localhost:8000/service/init/`, x)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
        })
    },
    //得到所有地图的基本信息
    getMaps() {
/*
      this.maps = [
        {
          map_id: 1,
          map_name: 'firmap',
          map_time: 20230507,
          map_remark: 'hello map!'
        },
      ]
 */
      this.$axios.post(`http://localhost:8000/map/showAll/`)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.maps = res.data.data
          }
        })

      return this.maps
    },
    // 导航
    submitNav() {
      console.log(this.navEndPoint)
      let x = {label_id: this.navEndPoint}
      this.$axios.post(`http://localhost:8000/navigation/begin/`, x)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.$message({
              message: '成功提交导航任务请求',
              type: 'success'
            });
          }
        })
    },
    // 抓取
    submitFetch() {
      let x = {label_id: this.catchEndPoint}
      this.$axios.post(`http://localhost:8000/object/fetch/`, x)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.$message({
              message: '成功提交导航任务请求',
              type: 'success'
            });
          }
        })
    },
    changeVoice() {
      this.$axios.post(`http://localhost:8000/control/voice/`)
        .then(res => {
          if (res.data.map_id == 400) {
            this.$message.error(res.data.msg)
          }
        })
    }
  },

}
</script>

<style scoped>

</style>
