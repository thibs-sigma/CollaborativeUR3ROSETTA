<template>
  <div class="hello">
    <v-container grid-list-xl justify-center>
      <h1>UR3 simulación - Interfaz de control > Eligir acción</h1>
      <p>Control de las funcionalidades de la demonstración con V-REP.</p>
      <v-data-table
        :headers="headers"
        :items="ur3simuNetwork"
        class="elevation-1"
        hide-actions
      >
        <template v-slot:items="props">
          <td>{{ props.item.topic }}</td>
          <td class="text-xs-left">{{ props.item.message_received }}</td>
        </template>
      </v-data-table>
      <!-- <p v-if="connected === '1'">Connected!</p> -->
      <br />

      <!-- BEGIN LINE 1 -->
      <v-layout align-center justify-center row wrap>
        <!-- MENU TILE 1 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(1)"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/demo_gears.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">FULL DEMO</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <!-- END MENU TILE 1 -->

        <!-- MENU TILE 2 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(2)"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/get_object.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">MANIP. PIEZA</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <!-- END MENU TILE 2 -->

        <!-- MENU TILE 3 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(3)"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/manip.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">MANIP. CONJUNTA</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <!-- END MENU TILE 3 -->
      </v-layout>
      <!-- END LINE 1 -->

      <!-- BEGIN LINE 2 -->
      <v-layout align-center justify-center row wrap>
        <!-- MENU TILE 4 -->

        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(4)"
          @click.stop="dialog = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/stop.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">STOP</span>
                <br />
                <!-- <span class="subheading">
                  Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <v-dialog v-model="dialog" persistent width="1200">
          <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >STOP ACTION</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!next"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'STOP' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else> <!-- Second message -->
                <v-icon size="100" color="black">playlist_add_check</v-icon>
                <br />
                <span class="headline font-weight-bold">
                  Ahora, seguir las acciones suiguientes:<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  <ul>
                    <li>1. Verificar que la simulación de V-REP se terminó</li> <br/>
                    <li>2. Cerrar la ventana de V-REP (si pregunta si debe guardar las modificaciones, eligir 'No')</li> <br/>
                    <li>3. Verificar la salida de consola en el Terminal <i>(Received shutdown request...)</i></li><br/>
                    <li>4. <i>Kill</i> todo, presionando <i>CTRL + C</i></li><br/>
                    <li>5. Cerrar esa applicación</li><br/>
                  </ul>
                </span>
                <br />
              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!next" color="orange" text dark @click="next = true"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialog = false;
                  next = false;
                "
                >Listo!</v-btn
              >
            </v-card-actions>
          </v-card>
        </v-dialog>
        <!-- END MENU TILE 4 -->

        <!-- MENU TILE 5 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(5)"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/reset.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">RESET</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <!-- END MENU TILE 5 -->
      </v-layout>
      <!-- END LINE 2 -->
    </v-container>
  </div>
</template>

<script>
/*eslint-disable*/
import ROS from "../ros_build/roslib.js";

var ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
// var ros_rds = new ROSLIB.Ros({ url: "ws://localhost:9090" });

// Publishing a Topic
// ------------------

// var cmdVel = new ROSLIB.Topic({
//   ros : ros,
//   name : '/cmd_vel',
//   messageType : 'geometry_msgs/Twist'
// });

// var twist = new ROSLIB.Message({
//   linear : {
//     x : 0.1,
//     y : 0.2,
//     z : 0.3
//   },
//   angular : {
//     x : -0.1,
//     y : -0.2,
//     z : -0.3
//   }
// });
// cmdVel.publish(twist);

// Subscribing to a Topic
// ----------------------

var desired_action = new ROSLIB.Topic({
  ros: ros,
  name: "/desired_action",
  messageType: "std_msgs/String"
});

var desired_action_data = new ROSLIB.Message({});

export default {
  name: "ur3simu_network",
  data() {
    return {
      dialog: false,
      next: false,
      end: false,

      connected: "",
      // TEST SIMPLE LISTENER
      // Initialize values
      desired_action: "",
      desired_action_data: "",

      headers: [
        {
          text: "Topics",
          align: "left",
          sortable: true,
          value: "name"
        },

        // NAME HEADERS
        { text: "Mensaje recibido", value: "message_received" }
      ],
      ur3simuNetwork: [
        {
          // Default text if nothing received (to be defined for each line)
          topic: "Ningun topic disponible...",
          message_received: "Ningun mensaje recibido..."
        }
      ]
    };
  },
  mounted() {
    this.init();
  },

  methods: {
    init() {
      ros.on("connection", function() {
        console.log("Connected to websocket server.");
        // var connected = '1';
      });

      ros.on("error", function(error) {
        console.log("Error connecting to websocket server: ", error);
        // var connected = 2;
      });

      ros.on("close", function() {
        console.log("Connection to websocket server closed.");
        // var connected = 0;
      });

      var self = this;

      // TEST
      desired_action.subscribe(function(message) {
        console.log(
          "Received message on " + desired_action.name + ": " + message.data
        );
        self.desired_action = message;
        // Echo on table
        self.ur3simuNetwork[0].topic = desired_action.name;
        self.ur3simuNetwork[0].message_received = self.desired_action.data;
      });
    },
    send_action(request_action_click) {
      // console.log(request_action_click);
      if (request_action_click == 1) {
        console.log("Requested action: 1 FULL");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "full";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: FULL");
      } else if (request_action_click == 2) {
        console.log("Requested action: 2 GET OBJ");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "get_object";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: get_object");
      } else if (request_action_click == 3) {
        console.log("Requested action: 3 MANIP");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "manip";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: manip");
      } else if (request_action_click == 4) {
        console.log("Requested action: 4 STOP");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "stop";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: stop");
      } else if (request_action_click == 5) {
        console.log("Requested action: 5 RESET");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "reset";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: reset");
      }
    }
  },
  computed: {
    binding() {
      const binding = {};
      if (this.$vuetify.breakpoint.mdAndUp) binding.column = true;
      return binding;
    }
  },
  props: {
    msg: String
  }
};
</script>

<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
</style>
