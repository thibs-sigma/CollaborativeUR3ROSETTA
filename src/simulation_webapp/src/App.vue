<template>
  <v-app>
    <Toolbar :menu="activeMenu" class="hidden-sm-and-down"></Toolbar>

    <v-content>
      <!-- <ur3simu /> -->
      <router-view />
    </v-content>

    <v-footer class="pa-3">
      <v-spacer></v-spacer>
      <div>ROSETTA Lab - UNCuyo &copy; {{ new Date().getFullYear() }}</div>
    </v-footer>
  </v-app>
</template>

<script>
// import ur3simu from "./components/simulation_network";
import Toolbar from "./components/Toolbar";

export default {
  name: "App",
  components: {
    // ur3simu,
    Toolbar
  },
  computed: {
    activeMenu() {
      var menu = [
      ];
      {
        menu.push({
          title: "Home",
          link: "/simulation_network",
          icon: "fa-pencil-alt"
        });
        menu.push({
          title: "Help",
          link: "/help",
          icon: "fa-pencil-alt"
        });
      }
      return menu;
    }
  }
};
</script>
