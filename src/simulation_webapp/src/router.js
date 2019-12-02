import Vue from "vue";
import Router from "vue-router";
// import store from "./store";

Vue.use(Router);

let router = new Router({
  mode: "history",
  routes: [
    {
      path: "/",
      name: "home",
      component: () => import(/* webpackChunkName: "home" */ "./views/Home.vue")
    },
    {
      path: "/simulation_network",
      name: "simulation_network",
      component: () =>
        import(
          /* webpackChunkName: "home" */ "./components/simulation_network.vue"
        )
    },
    {
      path: "/choose_action",
      name: "choose_action",
      component: () =>
        import(/* webpackChunkName: "home" */ "./components/choose_action.vue")
    }
  ]
});

router.beforeEach((to, from, next) => {
  next();
});

export default router;
