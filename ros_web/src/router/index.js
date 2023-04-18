import Vue from 'vue'
import Router from 'vue-router'
import MainPage from '../components/InnerPages/MainPage'
import Build from '../components/InnerPages/Build'
import Serve from '../components/InnerPages/Serve'
import Home from '../components/Home'
Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      component: Home,
      name: 'Home'
    },
    {
      path: '/MainPage',
      component: MainPage,
      name: 'MainPage'
    },
    {
      path: '/Build',
      component: Build,
      name: 'Build'
    },
    {
      path: '/Serve',
      component: Serve,
      name: 'Serve'
    }
  ]
})
