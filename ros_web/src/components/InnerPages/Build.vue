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
    <el-main>
      <el-row>
        <el-col :span="7" :gutter="20" v-for="(map, index) in maps" :key="map.mapId" style="padding-left: 4%">
<!--          地图card-->
          <el-card
            shadow="hover"
            class="card"
            style="width: 100%; margin-left: 5%;
            padding-left: 5%; margin-bottom: 6px; margin-right: 5%"
            @click.native="getDetail(map.mapId)">
            <el-row>
<!--             地图图片-->
              <el-col>
                <el-image :src="require(`@/assets/map.png`)" style="width: 100%"></el-image>
              </el-col>
            </el-row>
            <el-row style="padding-top: 10px">
<!--              地图名称-->
              <el-col style="text-align: center">
                <el-tag size="small" type="info" style="align-content: center">{{map.mapName}}</el-tag>
              </el-col>
            </el-row>
          </el-card>
        </el-col>
      </el-row>
<!--      新建地图弹窗-->
      <el-dialog title="新建地图" :visible.sync="dialogVisible" width="60%">
          <el-row style="height: 80%">
            新建地图
            <el-image
              style="width: 70%; height: 70%; padding-left: 8%"
              :src="require(`@/assets/map.png`)">
            </el-image>
          </el-row>
          <div slot="footer" class="dialog-footer">
            <el-button @click="dialogVisible = false">取 消</el-button>
            <el-button type="primary" @click="confirmCreate">确定提交</el-button>
          </div>
      </el-dialog>
<!--      地图信息编辑弹窗-->
      <el-dialog title="地图信息编辑" :visible.sync="dialogMark" width="60%">
        <el-row>
          <el-col :span="4">
            <span>航点标注</span>
          </el-col>
          <el-col :span="4" style="float: right">
            <el-button type="danger" icon="el-icon-delete" circle @click="dialogDelete = true"></el-button>
          </el-col>
        </el-row>
        <el-row>
          <el-image
            style="width: 70%; height: 70%; padding-left: 15%"
            :src="require(`@/assets/map.png`)">
          </el-image>
        </el-row>
        <div slot="footer" class="dialog-footer">
          <el-button @click="dialogMark = false">取 消</el-button>
          <el-button type="primary" @click="confirmMark">保存航点</el-button>
        </div>
      </el-dialog>
<!--      -->
      <el-dialog title="是否删除当前地图" :visible.sync="dialogDelete" >
        <el-row>
          <el-col :span="4" style="padding-left: 5%">
            <el-button @click="dialogDelete = false">取 消</el-button>
          </el-col>
          <el-col :span="4" style="margin-left: 65%">
            <el-button type="danger" @click="confirmDelete">确 认</el-button>
          </el-col>
        </el-row>
      </el-dialog>
    </el-main>
  </el-container>
</template>

<script>
/* eslint-disable */
import {methods} from 'babel-plugin-transform-runtime/lib/definitions'

export default {
  name: 'Build',
  data() {
    return {
      maps:[

      ],
      dialogVisible: false,
      dialogMark: false,
      dialogDelete: false,
      mapId: '1'
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
    getSRC(Id) {
      var dist = `/assets/${{Id}}.png`
      return dist
    },
    getDetail(mapId) {
      this.dialogMark = true
      this.mapId = mapId
    },
    getMaps() {
      return [
        {
          mapId:'1',
          mapName:'map1',
        },
        {
          mapId:'2',
          mapName:'map2',
        },
        {
          mapId:'3',
          mapName:'map3',
        }
        ]
    },
    createMap() {
      this.dialogVisible = true
    },
    confirmCreate() {
      //保存地图
      this.dialogVisible = false
    },
    confirmMark() {
      this.dialogMark = false
    },
    confirmDelete() {
      this.dialogDelete = false
    }
  },

}
</script>

<style scoped>

</style>
