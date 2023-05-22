<!-- eslint-disable -->
<template>
  <el-container>
    <el-header>
      <el-row :gutter="20" style="padding:0; margin: 10px 0 0 0">
        <el-col :span="6">
          <el-page-header @back="goBack" content="配置模式">
          </el-page-header>
        </el-col>
        <el-col :span="14">
          <space></space>
        </el-col>
        <el-col :span="4" style="float: right; align-content: center;text-align: center">
          <el-button type="text" @click="createMap">新建地图</el-button>
        </el-col>
      </el-row>
    </el-header>
<!--    地图部分-->
    <el-main>
      <el-row>
        <el-col :span="7" :gutter="20" v-for="(map, index) in maps" :key="map.map_id" style="padding-left: 4%">
<!--          地图card-->
          <el-card
            shadow="hover"
            class="card"
            style="width: 100%; margin-left: 5%;
            padding-left: 5%; margin-bottom: 6px; margin-right: 5%"
            @click.native="getDetail(map.map_id)">
            <el-row>
<!--             地图图片-->
              <el-col>
                <el-image :src="require(`//home//jinghongbin//SE//team03-project//src//patch_embedding//maps//map`+ map.map_id + `.png`)" style="width: 100%"></el-image>
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

<!--      新建地图弹窗-->
      <el-dialog title="新建地图中..." :visible.sync="dialogVisible" width="50%">
        <el-row style="height: 80%; margin-left: 15%">
          <el-image
            style="width: 70%; height: 70%; padding-left: 8%; align-content: center"
            :src="require(`@/assets/create_map.svg`)">
          </el-image>
        </el-row>
<!--        输入地图名称、备注-->
<!--        map_name-->
        <el-row style="margin-top: 10px; margin-left: 10px">
          <el-col :span="2" style="text-align: center; margin-top:10px;">
            名称：
          </el-col>
          <el-col :span="5">
            <el-input
              placeholder="请输入地图名称"
              v-model="map_name">
            </el-input>
          </el-col>
        </el-row>
<!--        map_remark-->
        <el-row style="margin-top: 10px; margin-left: 10px">
          <el-col :span="2" style="text-align: center; margin-top:10px;">
            备注：
          </el-col>
          <el-col :span="10">
            <el-input
              placeholder="请输入备注信息"
              v-model="map_remark">
            </el-input>
          </el-col>
        </el-row>
        <div slot="footer" class="dialog-footer">
          <el-button @click="dialogVisible = false">取 消</el-button>
          <el-button type="primary" @click.native="saveMap(map_name, map_remark)" style="position: relative">确认提交</el-button>
        </div>
      </el-dialog>

<!--      地图信息编辑弹窗 -->
      <el-dialog title="地图信息编辑" :visible.sync="dialogMark" width="60%">
        <el-row>
          <el-col :span="4" style="padding-left: 50px">
            <el-button round @click="createMark(map_id)">航点标记</el-button>
          </el-col>
          <el-col :span="4" style="float: right;">
            <el-button type="danger" icon="el-icon-delete" circle @click="dialogDelete = true"></el-button>
          </el-col>
        </el-row>
<!--    地图图片-->
        <el-row>
          <el-image
            style="width: 50%; height: 50%; padding-left: 25%"
            :src="require(`//home//jinghongbin//SE//team03-project//src//patch_embedding//maps//map`+ this.map_id + `.png`)">
          </el-image>
        </el-row>

<!--        航点显示-->
        <div>
          <h3 style="margin-left: 10%">已有标注</h3>
          <el-table :data="this.detail_info"
                    stripe style="width: 80%;
                     margin-left: 10%"
                    :header-cell-style="{'text-align':'center'}"
          >
            <el-table-column prop="label_id" label="航点id" witdh="100" align="center"></el-table-column>
            <el-table-column prop="label_name" label="航点名称" witdh="200" align="center"></el-table-column>
            <el-table-column prop="label_remark" label="备注信息" witdh="200" align="center"></el-table-column>
          </el-table>
        </div>
<!--        输入航点名称、备注-->
        <div style="margin-left: 7%; margin-top: 20px">
          <h3 style="margin-left: 3%">新建标注</h3>
  <!--        label_name-->
          <el-row style="margin-top: 10px;">
            <el-col :span="4" style="text-align: center; margin-top:10px;">
              航点名称：
            </el-col>
            <el-col :span="5">
              <el-input
                placeholder="请输入标注名称"
                v-model="label_name">
              </el-input>
            </el-col>
          </el-row>
  <!--        label_remark-->
          <el-row style="margin-top: 10px; ">
            <el-col :span="4" style="text-align: center; margin-top:10px;">
              航点备注：
            </el-col>
            <el-col :span="10">
              <el-input
                placeholder="请输入备注信息"
                v-model="label_remark">
              </el-input>
            </el-col>
          </el-row>
        </div>

        <div slot="footer" class="dialog-footer">
          <el-button @click="dialogMark = false">取 消</el-button>
          <el-button type="primary" @click="confirmMark(label_name, label_remark)" >保存航点</el-button>
        </div>
      </el-dialog>

<!--     删除地图弹窗 -->
      <el-dialog title="是否删除当前地图" :visible.sync="dialogDelete" >
        <el-row>
          <el-col :span="4" style="padding-left: 5%">
            <el-button @click="dialogDelete = false">取 消</el-button>
          </el-col>
          <el-col :span="4" style="margin-left: 65%">
            <el-button type="danger" @click="confirmDelete(map_id)">确 认</el-button>
          </el-col>
        </el-row>
      </el-dialog>

    </el-main>
  </el-container>
</template>

<script>
/* eslint-disable */
import {methods} from 'babel-plugin-transform-runtime/lib/definitions'
import Vue from "vue";

export default {
  name: 'Build',
  data() {
    return {
      // 所有地图
      maps:[],
      dialogVisible: false,
      dialogMark: false,
      dialogDelete: false,
      // 选定地图对应的信息
      map_id: 1,
      // 新建地图信息
      map_name: '',
      map_remark: '',
      // 新建标注信息
      label_name: '',
      label_remark: '',
      // 地图信息编辑中的标记
      // map_flag: false,
      // 指定地图的航点标记信息
      detail_info: []
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
    getSRC(url) {
      console.log(url)
      // return require(url)
    },
    // 显示指定地图中所有航点
    getDetail(map_id) {
      console.log(map_id)
      this.dialogMark = true
      this.map_id = map_id
      let x = {map_id: map_id}
      this.$axios.post(`http://localhost:8000/mark/show/`, x)
        .then(res => {
          if (res.data.code == 400) {
            this.$message.error(res.data.msg)
          }
          else {
            console.log(res.data.data)
            this.detail_info = res.data.data
          }
        })
/*
      this.detail_info = [
        {
          label_id: 1,
          label_name: 'label1',
          label_remark: 'hello'
        },
        {
          label_id: 2,
          label_name: 'label2',
          label_remark: 'hello 2'
        },
      ]
*/
    },

    // 显示所有地图
    getMaps() {
/*
      this.maps = [
        {
          map_id: 1,
          map_name: 'firmap',
          map_time: 20230507,
          map_remark: 'hello map!'
        },
        {
          map_id: 2,
          map_name: 'secmap',
          map_time: 20230823,
          map_remark: 'hello map two!'
        },
      ]
*/
      this.$axios.post(`http://localhost:8000/map/showAll/`)
        .then(res => {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.maps = res.data.data
          }
        })

      return this.maps
    },
    // 创建地图
    createMap() {
      this.dialogVisible = true
      this.$axios.post(`http://localhost:8000/map/create/`)
        .then(res=> {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
        })
    },


    //保存地图
    saveMap(map_name, map_remark) {
      console.log(map_name, map_remark)
      var map_info = {
        map_remark: map_remark,
        map_name: map_name
      }
      this.$axios.post(`http://localhost:8000/map/save/`, map_info)
        .then(res=> {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.getMaps()
            this.dialogVisible = false
            this.map_name = ''
            this.map_remark = ''
          }
        })
    },

    // 保存航点 mark/save
    confirmMark(label_name, label_remark) {
      var mark = {
        label_name: label_name,
        label_remark: label_remark
      }
      this.$axios.post(`http://localhost:8000/mark/save/`, mark)
        .then(res => {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
        })
      this.dialogMark = false
      this.label_name = ''
      this.label_remark = ''
    },

    // 创建航点
    createMark(map_id) {
      let x = {map_id:map_id}
      this.$axios.post(`http://localhost:8000/mark/create/`, x, {timeout:1000*60})
        .then(res=> {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
        })
    },
    // 删除地图
    confirmDelete(map_id) {
      let x = {map_id: map_id}
      this.$axios.post(`http://localhost:8000/map/delete/`,x)
        .then(res => {
          if (res.data.code === 400) {
            this.$message.error(res.data.msg)
          }
          else {
            this.maps = this.getMaps()
            // 重新加载地图
            this.dialogDelete = false
            this.dialogMark = false
          }
        })

    }
  },

}
</script>

<style scoped>

</style>
