<template>
  <v-app>
    <Toolbar></Toolbar>

    <v-content>
      <!-- <ur3simu /> -->
      <router-view />
    </v-content>

    <v-footer class="pa-3">
      <div>
        Algún problema? Contacta:
        <b>Thibaud.Hiltenbrand@sigma-clermont.fr</b>
      </div>
      <v-spacer></v-spacer>
      <div>ROSETTA Lab - UNCuyo &copy; {{ new Date().getFullYear() }}</div>
    </v-footer>
    <StatusConnexion />
  </v-app>
</template>

<script>
// import ur3simu from "./components/simulation_network";
import Toolbar from "./components/Toolbar";
import StatusConnexion from "./components/StatusConnexion";

export default {
  name: "App",
  components: {
    // ur3simu,
    Toolbar,
    StatusConnexion
  },
  computed: {}
};
</script>
