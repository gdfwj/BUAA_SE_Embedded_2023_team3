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
            inactive-text="语言控制">
          </el-switch>
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
            @click.native="getServe(map.mapId)">
            <el-row>
              <el-col>
                <el-image :src="require(`@/assets/map.png`)" style="width: 100%"></el-image>
              </el-col>
            </el-row>
            <el-row style="padding-top: 10px">
              <el-col style="text-align: center">
                <el-tag size="small" type="info" style="align-content: center">{{map.mapName}}</el-tag>
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
            style="width: 70%; height: 70%; padding-left: 15%"
            :src="require(`@/assets/map.png`)">
          </el-image>
        </el-row>
        <!--          手动标识-->
        <div v-if="switchValue == true">
          <!--        功能选择-->
          <el-row  :gutter="0">
            <el-col :span="7" style="padding-left: 5%">
              <el-select v-model="value" placeholder="请选择功能">
                <el-option
                  v-for="item in options"
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
              <el-col :span="4">
                初始位置：
              </el-col>
              <el-col :span="4">
                <el-select
                  v-model="navStartPoint"
                  placeholder="初始位置"
                  size="mini">
                  <el-option
                    v-for="item in optionsStart"
                    :key="item.value"
                    :label="item.label"
                    :value="item.value">
                  </el-option>
                </el-select>
              </el-col>
              <el-col :span="4">
                结束位置：
              </el-col>
              <el-col :span="4">
                <el-select
                  v-model="navEndPoint"
                  placeholder="结束位置"
                  size="mini">
                  <el-option
                    v-for="item in optionsEnd"
                    :key="item.value"
                    :label="item.label"
                    :value="item.value">
                  </el-option>
                </el-select>
              </el-col>
              <el-col :span="4" style="float: right">
                <el-button type="success" icon="el-icon-check" size="mini" circle @click="submitNav"></el-button>
              </el-col>
            </el-row>
          </div>
          <div v-if="value == 'label2'" style="text-align: center; align-content: center; align-items: center">
              <el-row style="padding-top: 20px">
                <el-col :span="4">
                  初始位置：
                </el-col>
                <el-col :span="4">
                  <el-select
                    v-model="catchStartPoint"
                    placeholder="初始位置"
                    size="mini">
                    <el-option
                      v-for="item in optionsStart"
                      :key="item.value"
                      :label="item.label"
                      :value="item.value">
                    </el-option>
                  </el-select>
                </el-col>
                <el-col :span="4">
                  目标位置：
                </el-col>
                <el-col :span="4">
                  <el-select
                    v-model="catchEndPoint"
                    placeholder="结束位置"
                    size="mini">
                    <el-option
                      v-for="item in optionsEnd"
                      :key="item.value"
                      :label="item.label"
                      :value="item.value">
                    </el-option>
                  </el-select>
                </el-col>
                <el-col :span="4" style="float: right">
                  <el-button type="success" icon="el-icon-check" size="mini" circle @click="submitNav"></el-button>
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
      maps:[{
        mapId: 1,
        mapName:'第一张地图',
      }],
      inputString: '',
      dialogServe: false,
      mapId: '1',
      value: '',
      switchValue: true,
      options: [
        {
          value: 'label1',
          label: '导航控制'
        },
        {
          value: 'label2',
          label: '取物控制'
        }
      ],
      navStartPoint: '当前位置',
      optionsStart: [
        {
          value: 'start1',
          label: '当前位置',
        },
        {
          value: 'start2',
          label: '航点1'
        }
      ],
      navEndPoint: '当前位置',
      optionsEnd: [
        {
          value: 'start1',
          label: '当前位置',
        },
        {
          value: 'start2',
          label: '航点1'
        }
      ],
      catchStartPoint: '当前位置',
      catchEndPoint: '当前位置'
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
      return `@/assets/${{Id}}.png`
    },
    getServe(mapId) {
      this.dialogServe = true
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
        },
      ]
    },
    submitNav() {
      this.$message({
        message: '成功提交导航任务请求',
        type: 'success'
      });
    }
  },

}
</script>

<style scoped>

</style>
