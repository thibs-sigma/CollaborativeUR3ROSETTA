<template>
  <div class="hello">
    <v-container>
      <h1>UR3 simulación - Interfaz de control > Ínicio</h1>
      <p>Control de las funcionalidades de la demonstración con V-REP.</p>

      <br />

      <!-- BEGIN LINE 1 -->
      <v-layout align-center justify-center row> 

        <!-- MENU TILE 1 -->

        <v-card height="270" width="300" hover ripple class="mx-3 my-2" to="choose_action">
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <a href="choose_action" style="text-decoration: none; color: black">
                <v-flex xs12>
                  <p>
                    <img :src="require('../assets/img/control.png')" style="height: 150px; margin-top: 35px; margin-bottom: -30px;" />
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

        <!-- MENU TILE 2 -->

        <v-card height="270" width="300" hover ripple class="mx-3 my-2" to="simulation_network">
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <a href="simulation_network" style="text-decoration: none; color: black">
                <v-flex xs12>
                  <p>
                    <img :src="require('../assets/img/network.png')" style="height: 150px; margin-top: 35px; margin-bottom: -30px;" />
                  </p>
                  <br />
                  <span class="headline font-weight-bold">RED DE ROS</span>
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

        <!-- END MENU TILE 2 -->
      </v-layout>

      <!-- BEGIN LINE 2 -->
      <v-layout align-center justify-center row> 

      <!-- MENU TILE 3 -->

      <v-card height="270" width="300" hover ripple class="mx-3 my-2" to="https://github.com/thibs-sigma/CollaborativeUR3ROSETTA/wiki">
        <v-container grid-list-md text-xs-center>
          <v-layout align-center justify-center fill-height>
            <a href="https://github.com/thibs-sigma/CollaborativeUR3ROSETTA/wiki/Interfaz-gráfica" target="_blank" style="text-decoration: none; color: black">
              <v-flex xs12>
                <p>
                  <img :src="require('../assets/img/help.png')" style="height: 150px; margin-top: 35px; margin-bottom: -30px;" />
                </p>
                <br />
                <span class="headline font-weight-bold">AYUDA</span>
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

      <!-- END MENU TILE 3 -->

      </v-layout>

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
