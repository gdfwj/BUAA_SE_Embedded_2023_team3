// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
/* eslint-disable */
import Vue from 'vue'
import App from './App'
import router from './router'
import axios from 'axios'
import Element from 'element-ui'
import VueParticles from 'vue-particles'
Vue.use(VueParticles)

Vue.use(Element)
Vue.config.productionTip = false
Vue.prototype.$axios = axios
axios.default.baseUrl = 'http://localhost:8080'
/* eslint-disable no-new */
new Vue({
  el: '#app',
  router,
  components: { App },
  template: '<App/>'
})
