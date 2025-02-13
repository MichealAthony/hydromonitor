// Composables
import { createRouter, createWebHistory } from 'vue-router'

const routes = [
  { path: '/', name: 'Home', component: () => import('@/views/Home.vue'), meta:{ transition: 'fade'} }, 
  // ADD NEW ROUTES HERE
  { path: '/Control', name: 'Control', component: () => import('@/views/Control.vue'), meta:{ transition: 'fade'} }, 

  { path: '/Live', name: 'Live', component: () => import('@/views/Live.vue'), meta:{ transition: 'fade'} }, 

  { path: '/Analysis', name: 'Analysis', component: () => import('@/views/Analysis.vue'), meta:{ transition: 'fade'} }, 


]

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
})

export default router
