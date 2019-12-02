<template>
  <div class="hello">
    <v-container>
      <h1>UR3 simulación - Interfaz de control > Ínicio</h1>
      <p>Control de las funcionalidades de la demonstración con V-REP.</p>

      <br />

      <!-- MENU TILE 1 -->

      <v-card height="270" width="300" hover ripple class="mx-3 my-2" to="choose_action">
        <v-container grid-list-md text-xs-center>
          <v-layout align-center justify-center fill-height>
            <a href="choose_action" style="text-decoration: none; color: black">
              <v-flex xs12>
                <p>
                  <img :src="require('../assets/logo.png')" style="height: 150px;" />
                </p>
                <br />
                <span class="headline font-weight-bold">CONTROL DE LA SIMULACION</span>
                <br />
                <!-- <span class="subheading">
                  Test 3
                </span>-->
                <br />
              </v-flex>
            </a>
          </v-layout>
        </v-container>
      </v-card>

      <!-- END MENU TILE 1 -->
    </v-container>
  </div>
</template>

<script>
/*eslint-disable*/
import ROS from "../ros_build/roslib.js";

var ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

export default {
  name: "ur3simu_network",
  data() {},
  mounted() {
    this.init();
  },

  methods: {
    init() {}
  },
  computed: {},
  props: {}
};
</script>
